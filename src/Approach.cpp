#include "executor_tools/Approach.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

namespace {

struct ApproachTarget {
  std::string category;
  std::string index;
};

bool hasNumericSuffix(const std::string &value, std::size_t sep) {
  if (sep == std::string::npos || sep + 1 >= value.size())
    return false;

  return std::all_of(value.begin() + sep + 1, value.end(),
                     [](unsigned char c) { return std::isdigit(c); });
}

ApproachTarget resolveTarget(const std::string &argument) {
  ApproachTarget target;

  if (argument == "person" || argument == "customer") {
    target.category = "customer";
    target.index = "1";
    return target;
  }

  if (argument == "host") {
    target.category = "host";
    target.index = "1";
    return target;
  }

  const std::size_t sep = argument.rfind('_');
  if (hasNumericSuffix(argument, sep)) {
    target.category = argument.substr(0, sep);
    target.index = argument.substr(sep + 1);
  } else {
    target.category = argument;
    target.index = "1";
  }

  if (target.category == "person")
    target.category = "customer";

  return target;
}

bool navigationArrayToPose(const json &navigation,
                           geometry_msgs::msg::PoseStamped &pose) {
  if (!navigation.is_array() || navigation.size() < 7)
    return false;

  pose.pose.position.x = navigation.at(0).get<double>();
  pose.pose.position.y = navigation.at(1).get<double>();
  pose.pose.position.z = navigation.at(2).get<double>();
  pose.pose.orientation.x = navigation.at(3).get<double>();
  pose.pose.orientation.y = navigation.at(4).get<double>();
  pose.pose.orientation.z = navigation.at(5).get<double>();
  pose.pose.orientation.w = navigation.at(6).get<double>();

  const double q_norm =
      std::sqrt(pose.pose.orientation.x * pose.pose.orientation.x +
                pose.pose.orientation.y * pose.pose.orientation.y +
                pose.pose.orientation.z * pose.pose.orientation.z +
                pose.pose.orientation.w * pose.pose.orientation.w);

  return q_norm > 0.1;
}

std::string jsonStringOrDefault(const json &value,
                                const std::string &fallback) {
  if (value.is_string())
    return value.get<std::string>();
  return fallback;
}

} // namespace

// =========================================================
// Constructor
// =========================================================
Approach::Approach(const rclcpp::Node::SharedPtr &node,
                   const std::shared_ptr<Hardware> &hardware,
                   const std::shared_ptr<Vision> &vision,
                   const std::shared_ptr<Navigation> &navigation,
                   const std::shared_ptr<Knowledge> &knowledge)
    : node_(node), hardware_(hardware), vision_(vision),
      navigation_(navigation), knowledge_(knowledge) {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  global_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/nav_control/goal", 10);

  stop_nav_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/navigation/navigation/stop", 10);
}

// =========================================================
// run
// =========================================================
Result Approach::run(const std::string &argument,
                     const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active())
    return {false, "FAILED", "goal_handle_inactive"};

  publishFeedback(gh, ApproachState::READY, argument);

  const ApproachTarget target = resolveTarget(argument);

  RCLCPP_INFO(node_->get_logger(),
              "[Approach] Querying Knowledge for key: %s_%s",
              target.category.c_str(), target.index.c_str());

  publishFeedback(gh, ApproachState::LOADING, argument);

  const json navigation_json =
      knowledge_->get_world_state(target.category, target.index, "navigation");

  geometry_msgs::msg::PoseStamped nav_pose;
  if (!navigationArrayToPose(navigation_json, nav_pose)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[Approach] Empty or invalid navigation array for target: %s_%s",
                 target.category.c_str(), target.index.c_str());
    publishFeedback(gh, ApproachState::FAILED, "invalid_knowledge_data");
    return {false, "FAILED", "invalid_knowledge_data"};
  }

  const json frame_json =
      knowledge_->get_world_state(target.category, target.index,
                                  "navigation_frame");
  nav_pose.header.frame_id = jsonStringOrDefault(frame_json, "base_link");
  nav_pose.header.stamp = node_->now();

  geometry_msgs::msg::PoseStamped map_goal;
  try {
    if (nav_pose.header.frame_id == "map") {
      map_goal = nav_pose;
      map_goal.header.stamp = node_->now();
    } else {
      const auto tf = tf_buffer_->lookupTransform(
          "map", nav_pose.header.frame_id, tf2::TimePointZero, 500ms);
      tf2::doTransform(nav_pose, map_goal, tf);
      map_goal.header.frame_id = "map";
      map_goal.header.stamp = node_->now();
    }
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[Approach] TF transform to map failed: %s", ex.what());
    publishFeedback(gh, ApproachState::FAILED, "tf_transform_error");
    return {false, "FAILED", "tf_transform_error"};
  }

  publishFeedback(gh, ApproachState::APPROACHING,
                  "publishing_navigation_goal");

  constexpr double kArrivalTolerance = 0.45;
  const auto kMaxNavDuration = 120s;
  auto start_nav_time = std::chrono::steady_clock::now();
  auto last_goal_time = std::chrono::steady_clock::now() - 5s;

  while (gh->is_active()) {
    const auto now_time = std::chrono::steady_clock::now();

    if (now_time - last_goal_time > 3s) {
      map_goal.header.stamp = node_->now();
      global_goal_pub_->publish(map_goal);
      last_goal_time = now_time;
      RCLCPP_INFO(node_->get_logger(),
                  "[Approach] Published navigation goal in map: [%.3f, %.3f]",
                  map_goal.pose.position.x, map_goal.pose.position.y);
    }

    try {
      const auto tf_robot = tf_buffer_->lookupTransform(
          "map", "base_link", tf2::TimePointZero, 100ms);

      const double dx =
          map_goal.pose.position.x - tf_robot.transform.translation.x;
      const double dy =
          map_goal.pose.position.y - tf_robot.transform.translation.y;
      const double distance_to_goal = std::sqrt(dx * dx + dy * dy);

      if (distance_to_goal <= kArrivalTolerance) {
        std_msgs::msg::Empty stop_msg;
        stop_nav_pub_->publish(stop_msg);
        RCLCPP_INFO(node_->get_logger(),
                    "[Approach] Goal reached. Distance error: %.3f m",
                    distance_to_goal);
        publishFeedback(gh, ApproachState::SUCCESS, "arrived_successfully");
        return {true, "SUCCESS", "arrived_successfully"};
      }
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[Approach] Waiting for localization update: %s",
                           ex.what());
    }

    if (now_time - start_nav_time > kMaxNavDuration) {
      std_msgs::msg::Empty stop_msg;
      stop_nav_pub_->publish(stop_msg);
      RCLCPP_ERROR(node_->get_logger(),
                   "[Approach] Navigation timeout reached");
      publishFeedback(gh, ApproachState::FAILED, "navigation_timeout");
      return {false, "FAILED", "navigation_timeout"};
    }

    std::this_thread::sleep_for(100ms);
  }

  std_msgs::msg::Empty stop_msg;
  stop_nav_pub_->publish(stop_msg);
  publishFeedback(gh, ApproachState::FAILED, "goal_cancelled");
  return {false, "FAILED", "goal_cancelled"};
}

// =========================================================
// Métodos Auxiliares
// =========================================================
void Approach::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                               ApproachState state,
                               const std::string &detail) {
  if (!gh || !gh->is_active())
    return;

  std::string msg = "[approach] " + stateName(state);
  if (!detail.empty())
    msg += ": " + detail;

  auto fb = std::make_shared<ExecuteTask::Feedback>();
  fb->feedback.message = msg;
  gh->publish_feedback(fb);

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

std::string Approach::stateName(ApproachState s) {
  switch (s) {
  case ApproachState::READY:
    return "READY";
  case ApproachState::LOADING:
    return "LOADING";
  case ApproachState::APPROACHING:
    return "APPROACHING";
  case ApproachState::SUCCESS:
    return "SUCCESS";
  case ApproachState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}
