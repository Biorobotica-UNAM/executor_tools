#include "executor_tools/FindPose.hpp"
#include "executor_tools/Result.hpp"

#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace {

geometry_msgs::msg::PoseStamped
navigationPoseFromPoint(const geometry_msgs::msg::PointStamped &point,
                        double scale) {
  const double px = point.point.x;
  const double py = point.point.y;

  const double nav_x = px * scale;
  const double nav_y = py * scale;

  const double yaw = std::atan2(py, px);

  geometry_msgs::msg::PoseStamped nav;
  nav.header = point.header;
  nav.pose.position.x = nav_x;
  nav.pose.position.y = nav_y;
  nav.pose.position.z = 0.0;
  nav.pose.orientation.x = 0.0;
  nav.pose.orientation.y = 0.0;
  nav.pose.orientation.z = std::sin(yaw / 2.0);
  nav.pose.orientation.w = std::cos(yaw / 2.0);

  return nav;
}

} // namespace

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────

FindPose::FindPose(const rclcpp::Node::SharedPtr &node,
                   const std::shared_ptr<Hardware> &hardware,
                   const std::shared_ptr<HRI> &hri,
                   const std::shared_ptr<Knowledge> &knowledge,
                   const PoseBuffer &customer_pose_map)
    : node_(node),
      hardware_(hardware),
      hri_(hri),
      knowledge_(knowledge),
      customer_pose_map_(customer_pose_map) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

// ─────────────────────────────────────────────────────────────
//  run
// ─────────────────────────────────────────────────────────────

Result FindPose::run(const std::string &argument,
                     const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active())
    return {false, "FAILED", "goal_inactive"};

  publishFeedback(gh, FindPoseState::READY, argument);

  if (argument.empty()) {
    publishFeedback(gh, FindPoseState::FAILED, "empty_argument");
    return {false, "FAILED", "empty_argument"};
  }

  std::string action_type;
  std::string human_index;
  parseArgument(argument, action_type, human_index);

  for (const auto &[pan, tilt] : scan_poses_) {
    if (!gh->is_active())
      return {false, "FAILED", "goal_cancelled"};

    publishFeedback(gh, FindPoseState::MOVING_HEAD);
    hardware_->MoveHead({pan, tilt});
    std::this_thread::sleep_for(1500ms);

    publishFeedback(gh, FindPoseState::STABILIZING);
    std::this_thread::sleep_for(500ms);

    publishFeedback(gh, FindPoseState::DETECTING, action_type);

    std::string out_direction;
    geometry_msgs::msg::PointStamped out_point;

    const bool success =
        hri_->FindPose(action_type, "any", out_direction, out_point);

    if (success) {

      RCLCPP_INFO(
      node_->get_logger(),
      "[find_pose] received point: frame=%s x=%.3f y=%.3f z=%.3f",
      out_point.header.frame_id.c_str(),
      out_point.point.x,
      out_point.point.y,
      out_point.point.z);

      publishFeedback(gh, FindPoseState::LOOKING_AT_TARGET, out_direction);
      hardware_->LookAt(out_point);
      std::this_thread::sleep_for(1500ms);

      publishFeedback(gh, FindPoseState::UPDATING, out_direction);
      const auto nav = navigationPoseFromPoint(out_point, 0.8);

      RCLCPP_INFO(
      node_->get_logger(),
      "[find_pose] navigation pose: x=%.3f y=%.3f z=%.3f q=[%.3f %.3f %.3f %.3f]",
      nav.pose.position.x,
      nav.pose.position.y,
      nav.pose.position.z,
      nav.pose.orientation.x,
      nav.pose.orientation.y,
      nav.pose.orientation.z,
      nav.pose.orientation.w);

      updateHumanPose(action_type, human_index, out_direction, out_point, nav);
      updateCustomerPoseBuffer(out_point);

      publishFeedback(gh, FindPoseState::SUCCESS, out_direction);
      return {true, "SUCCESS",
              "human_" + action_type + "_" + human_index + "_found_at_" +
                  out_direction};
    }
  }

  publishFeedback(gh, FindPoseState::FAILED, action_type);
  return {false, "FAILED", action_type + "_pose_not_found"};
}

// ─────────────────────────────────────────────────────────────
//  parseArgument
// ─────────────────────────────────────────────────────────────

bool FindPose::parseArgument(const std::string &argument,
                             std::string &out_action,
                             std::string &out_index) const {
  const std::size_t sep = argument.rfind('_');

  if (sep == std::string::npos || sep == 0 || sep + 1 == argument.size()) {
    out_action = argument;
    out_index = "1";
    return false;
  }

  out_action = argument.substr(0, sep);
  out_index = argument.substr(sep + 1);
  return true;
}

// ─────────────────────────────────────────────────────────────
//  updateHumanPose
// ─────────────────────────────────────────────────────────────

void FindPose::updateHumanPose(const std::string &action_type,
                               const std::string &index,
                               const std::string &direction,
                               const geometry_msgs::msg::PointStamped &point,
                               const geometry_msgs::msg::PoseStamped &nav) {
  const std::string category = "customer";

  nlohmann::json position = {point.point.x, point.point.y, point.point.z};
  const auto &np = nav.pose.position;
  const auto &nq = nav.pose.orientation;
  nlohmann::json navigation = {np.x, np.y, np.z, nq.x, nq.y, nq.z, nq.w};

  //knowledge_->update_world_state(category, index, "action", action_type);
  //knowledge_->update_world_state(category, index, "direction_zone", direction);
  //knowledge_->update_world_state(category, index, "frame_id",
                                 //point.header.frame_id.empty() ? "base_link"
                                                               //: point.header.frame_id);
  //knowledge_->update_world_state(category, index, "position", position.dump());
  knowledge_->update_world_state(category, index, "navigation",
                                 navigation.dump());
  //knowledge_->update_world_state(
      //category, index, "navigation_frame",
      //nav.header.frame_id.empty() ? "base_link" : nav.header.frame_id);
}

void FindPose::updateCustomerPoseBuffer(
    const geometry_msgs::msg::PointStamped &point) {
  if (!customer_pose_map_) {
    return;
  }

  geometry_msgs::msg::PointStamped point_in_map = point;
  if (point_in_map.header.frame_id.empty()) {
    point_in_map.header.frame_id = "base_link";
  }

  try {
    if (point_in_map.header.frame_id != "map") {
      const auto transform = tf_buffer_->lookupTransform(
          "map", point_in_map.header.frame_id, tf2::TimePointZero, 500ms);
      geometry_msgs::msg::PointStamped transformed_point;
      tf2::doTransform(point_in_map, transformed_point, transform);
      point_in_map = transformed_point;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = point_in_map.header;
    pose.header.stamp = node_->now();
    pose.pose.position = point_in_map.point;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    *customer_pose_map_ = pose;

    RCLCPP_INFO(node_->get_logger(),
                "[find_pose] buffered customer pose in map: [%.3f %.3f %.3f]",
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z);
  } catch (const tf2::TransformException &ex) {
    customer_pose_map_->reset();
    RCLCPP_WARN(node_->get_logger(),
                "[find_pose] could not transform detected point from '%s' to map: %s",
                point_in_map.header.frame_id.c_str(),
                ex.what());
  }
}

// ─────────────────────────────────────────────────────────────
//  publishFeedback
// ─────────────────────────────────────────────────────────────

void FindPose::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                               FindPoseState state, const std::string &detail) {
  if (!gh->is_active())
    return;

  std::string msg = "[find_pose] " + stateName(state);
  if (!detail.empty())
    msg += ": " + detail;

  auto fb = std::make_shared<ExecuteTask::Feedback>();
  fb->feedback.message = msg;
  gh->publish_feedback(fb);

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// ─────────────────────────────────────────────────────────────
//  stateName
// ─────────────────────────────────────────────────────────────

std::string FindPose::stateName(FindPoseState s) {
  switch (s) {
  case FindPoseState::READY:
    return "READY";
  case FindPoseState::MOVING_HEAD:
    return "MOVING_HEAD";
  case FindPoseState::STABILIZING:
    return "STABILIZING";
  case FindPoseState::DETECTING:
    return "DETECTING";
  case FindPoseState::LOOKING_AT_TARGET:
    return "LOOKING_AT_TARGET";
  case FindPoseState::UPDATING:
    return "UPDATING";
  case FindPoseState::SUCCESS:
    return "SUCCESS";
  case FindPoseState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}
