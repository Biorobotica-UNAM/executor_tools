#include "executor_tools/SaveLoc.hpp"

#include <algorithm>
#include <cctype>
#include <utility>
#include <chrono>
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

SaveLoc::SaveLoc(const rclcpp::Node::SharedPtr &node,
                 const std::shared_ptr<Navigation> &navigation,
                 const PoseBuffer &customer_pose_map)
    : node_(node),
      navigation_(navigation),
      customer_pose_map_(customer_pose_map) 
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Result SaveLoc::run(const std::string &location_name,
                    const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active()) {
    return {false, "FAILED", "goal_handle_inactive"};
  }

  const std::string name = trim(location_name);
  publishFeedback(gh, SaveLocState::READY, name);

  if (name.empty()) {
    publishFeedback(gh, SaveLocState::FAILED, "empty_location_name");
    return {false, "FAILED", "not_saved:empty_location_name"};
  }

  publishFeedback(gh, SaveLocState::SAVING, name);
  
  const std::string normalized_name = normalize(name);
  const bool save_customer_pose = normalized_name == "customer";
  const bool save_kitchen_bar = normalized_name == "kitchen_bar";

  // ── CASO A: GUARDAR CLIENTE ─────────────────────────────────────
  if (save_customer_pose) {
    if (!customer_pose_map_ || !customer_pose_map_->has_value()) {
      publishFeedback(gh, SaveLocState::FAILED, "customer_pose_missing");
      return {false, "FAILED", "not_saved:customer_pose_missing"};
    }

    const auto &pose_stamped = customer_pose_map_->value();

    if (pose_stamped.header.frame_id == "map") {
      const bool saved =
          navigation_->SavePoseLocation(name, pose_stamped.pose);

      if (saved) {
        publishFeedback(gh, SaveLocState::SUCCESS, name);
        return {true, "SUCCESS", "saved:" + name};
      }

      publishFeedback(gh, SaveLocState::FAILED, "pose_save_failed");
      return {false, "FAILED", "not_saved:" + name};
    }

    RCLCPP_WARN(
        node_->get_logger(),
        "[saveloc] customer pose for '%s' is in frame '%s', expected 'map'",
        name.c_str(),
        pose_stamped.header.frame_id.c_str());

    publishFeedback(gh, SaveLocState::FAILED, "customer_pose_not_in_map");
    return {false, "FAILED", "not_saved:customer_pose_not_in_map"};
  }
  
  // ── CASO B: CASO ESPECIAL KITCHEN_BAR (+90° a la Izquierda) ──────
  else if (save_kitchen_bar) {
    geometry_msgs::msg::Pose robot_pose;

    try {
      auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, 200ms);

      robot_pose.position.x = tf.transform.translation.x;
      robot_pose.position.y = tf.transform.translation.y;
      robot_pose.position.z = tf.transform.translation.z;

      tf2::Quaternion q_orig(
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z,
          tf.transform.rotation.w);

      tf2::Quaternion q_rot;
      q_rot.setRPY(0.0, 0.0, 1.57079632679);

      tf2::Quaternion q_new = q_orig * q_rot;
      q_new.normalize();

      robot_pose.orientation.x = q_new.x();
      robot_pose.orientation.y = q_new.y();
      robot_pose.orientation.z = q_new.z();
      robot_pose.orientation.w = q_new.w();

      RCLCPP_INFO(node_->get_logger(), 
                  "[saveloc] Intercepted 'kitchen_bar'. Rotated robot pose +90 deg over Z axis.");

    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "[saveloc] Failed to get robot pose for kitchen_bar: %s", ex.what());
      publishFeedback(gh, SaveLocState::FAILED, "tf_lookup_failed");
      return {false, "FAILED", "not_saved:tf_lookup_failed"};
    }

    const bool saved = navigation_->SavePoseLocation(name, robot_pose);
    if (saved) {
      publishFeedback(gh, SaveLocState::SUCCESS, name);
      return {true, "SUCCESS", "saved:" + name};
    }

    publishFeedback(gh, SaveLocState::FAILED, "pose_save_failed");
    return {false, "FAILED", "not_saved:" + name};
  }

  // ── CASO C: UBICACIÓN ESTÁNDAR (AddLocation ordinario) ───────────
  const bool saved = navigation_->AddLocation(name);
  if (saved) {
    publishFeedback(gh, SaveLocState::SUCCESS, name);
    return {true, "SUCCESS", "saved:" + name};
  }

  publishFeedback(gh, SaveLocState::FAILED, name);
  return {false, "FAILED", "not_saved:" + name};
}

void SaveLoc::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                              SaveLocState state,
                              const std::string &detail) const {
  if (!gh->is_active()) {
    return;
  }

  std::string msg = "[saveloc] " + stateName(state);
  if (!detail.empty()) {
    msg += ": " + detail;
  }

  auto fb = std::make_shared<ExecuteTask::Feedback>();
  fb->feedback.message = msg;
  gh->publish_feedback(fb);

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

std::string SaveLoc::stateName(SaveLocState state) {
  switch (state) {
    case SaveLocState::READY:   return "READY";
    case SaveLocState::SAVING:  return "SAVING";
    case SaveLocState::SUCCESS: return "SUCCESS";
    case SaveLocState::FAILED:  return "FAILED";
    default:                    return "UNKNOWN";
  }
}

std::string SaveLoc::trim(std::string value) {
  auto is_not_space = [](unsigned char c) {
    return !std::isspace(c);
  };

  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(), is_not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), is_not_space).base(),
              value.end());
  return value;
}

std::string SaveLoc::normalize(std::string value) {
  value = trim(std::move(value));
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  return value;
}