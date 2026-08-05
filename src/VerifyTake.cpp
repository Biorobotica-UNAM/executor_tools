#include "executor_tools/VerifyTake.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cctype>
#include <limits>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;

namespace
{

std::string owlPromptFromCategory(std::string category)
{
  std::replace(category.begin(), category.end(), '_', ' ');
  return category;
}

}  // namespace

VerifyTake::VerifyTake(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<Vision> & vision,
  const std::shared_ptr<Knowledge> & knowledge)
: node_(node),
  vision_(vision),
  knowledge_(knowledge)
{
  node_->declare_parameter<double>(
    "verify_take_still_present_distance_m",
    still_present_distance_m_);
  node_->get_parameter(
    "verify_take_still_present_distance_m",
    still_present_distance_m_);
}

Result VerifyTake::run(
  const std::string & argument,
  const std::shared_ptr<GoalHandle> & gh)
{
  if (!gh || !gh->is_active()) {
    return {false, "FAILED", "goal_inactive"};
  }

  std::string category;
  std::string index;
  if (!parseArgument(argument, category, index)) {
    return {false, "FAILED", "invalid_argument"};
  }

  const nlohmann::json previous_pose =
    knowledge_->get_world_state(category, index, "manipulation");
  if (!previous_pose.is_array() || previous_pose.size() < 3) {
    return {false, "FAILED", "take_uncertain:" + argument};
  }

  const double old_x = previous_pose.at(0).get<double>();
  const double old_y = previous_pose.at(1).get<double>();
  const double old_z = previous_pose.at(2).get<double>();

  auto detections = vision_->DetectOwlV2({owlPromptFromCategory(category)}, 5s);
  if (!detections.first) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[VerifyTake] OWL service failed while verifying '%s'",
      argument.c_str());
    return {false, "FAILED", "take_uncertain:" + argument};
  }

  if (detections.second.empty()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[VerifyTake] no '%s' detections near previous pose; take verified",
      argument.c_str());
    return {true, "SUCCESS", "take_verified:" + argument};
  }

  double nearest_distance = std::numeric_limits<double>::infinity();
  std::string nearest_name;
  int converted_detections = 0;

  for (const auto & detection : detections.second) {
    geometry_msgs::msg::PoseStamped detected_pose;
    geometry_msgs::msg::PoseStamped detected_nav;
    if (!vision_->DetectionToNavPose(detection, detected_pose, detected_nav, 3s)) {
      continue;
    }

    ++converted_detections;
    const double dx = detected_pose.pose.position.x - old_x;
    const double dy = detected_pose.pose.position.y - old_y;
    const double dz = detected_pose.pose.position.z - old_z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_name = detection.class_name;
    }
  }

  if (converted_detections == 0) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[VerifyTake] detections for '%s' could not be converted to poses",
      argument.c_str());
    return {false, "FAILED", "take_uncertain:" + argument};
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[VerifyTake] object='%s' nearest_detected='%s' distance_to_previous=%.3f threshold=%.3f converted=%d total=%zu",
    argument.c_str(),
    nearest_name.c_str(),
    nearest_distance,
    still_present_distance_m_,
    converted_detections,
    detections.second.size());

  if (nearest_distance <= still_present_distance_m_) {
    clearHeldObjectIfMatches(argument);
    return {false, "FAILED", "take_failed_object_still_present:" + argument};
  }

  return {true, "SUCCESS", "take_verified:" + argument};
}

bool VerifyTake::parseArgument(
  const std::string & argument,
  std::string & category,
  std::string & index) const
{
  const std::size_t sep = argument.rfind('_');
  if (sep == std::string::npos || sep == 0 || sep + 1 >= argument.size()) {
    return false;
  }

  category = argument.substr(0, sep);
  index = argument.substr(sep + 1);
  return std::all_of(
    index.begin(),
    index.end(),
    [](unsigned char c) {return std::isdigit(c) != 0;});
}

void VerifyTake::clearHeldObjectIfMatches(const std::string & object_id) const
{
  const auto clear_if_matches = [this, &object_id](const std::string & hand)
    {
      const nlohmann::json held = knowledge_->get_world_state("robot", "", hand);
      if (!held.is_string()) {
        return;
      }

      if (held.get<std::string>() != object_id) {
        return;
      }

      const bool updated =
        knowledge_->update_world_state("robot", "", hand, "nothing");
      if (updated) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[VerifyTake] cleared stale %s occupancy for '%s'",
          hand.c_str(),
          object_id.c_str());
      } else {
        RCLCPP_WARN(
          node_->get_logger(),
          "[VerifyTake] failed to clear stale %s occupancy for '%s'",
          hand.c_str(),
          object_id.c_str());
      }
    };

  clear_if_matches("left_hand");
  clear_if_matches("right_hand");
}
