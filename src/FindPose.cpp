#include "executor_tools/FindPose.hpp"
#include "executor_tools/Result.hpp"

#include <chrono>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────

FindPose::FindPose(const rclcpp::Node::SharedPtr &node,
                   const std::shared_ptr<Hardware> &hardware,
                   const std::shared_ptr<HRI> &hri,
                   const std::shared_ptr<Knowledge> &knowledge)
    : node_(node), hardware_(hardware), hri_(hri), knowledge_(knowledge) {}

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

  for (const auto &[pan, tilt] : scan_poses_) {
    if (!gh->is_active())
      return {false, "FAILED", "goal_cancelled"};

    publishFeedback(gh, FindPoseState::MOVING_HEAD);
    hardware_->MoveHead({pan, tilt});
    std::this_thread::sleep_for(1500ms);

    publishFeedback(gh, FindPoseState::STABILIZING);
    std::this_thread::sleep_for(500ms);

    publishFeedback(gh, FindPoseState::DETECTING, argument);

    std::string out_direction;
    geometry_msgs::msg::PointStamped out_point;

    const bool success =
        hri_->FindPose(argument, "any", out_direction, out_point);

    if (success) {
      publishFeedback(gh, FindPoseState::LOOKING_AT_TARGET, out_direction);
      hardware_->LookAt(out_point);
      std::this_thread::sleep_for(1500ms);

      publishFeedback(gh, FindPoseState::UPDATING, out_direction);
      updateHumanPose(argument, out_direction, out_point);

      publishFeedback(gh, FindPoseState::SUCCESS, out_direction);
      return {true, "SUCCESS",
              "human_" + argument + "_found_at_" + out_direction};
    }
  }

  publishFeedback(gh, FindPoseState::FAILED, argument);
  return {false, "FAILED", argument + "_pose_not_found"};
}

// ─────────────────────────────────────────────────────────────
//  updateHumanPose
// ─────────────────────────────────────────────────────────────

void FindPose::updateHumanPose(const std::string &action_type,
                               const std::string &direction,
                               const geometry_msgs::msg::PointStamped &point) {
  const std::string category = "detected_human";
  const std::string id = "1";

  nlohmann::json position = {point.point.x, point.point.y, point.point.z};

  knowledge_->update_world_state(category, id, "action", action_type);
  knowledge_->update_world_state(category, id, "direction_zone", direction);
  knowledge_->update_world_state(category, id, "frame_id",
                                 point.header.frame_id);
  knowledge_->update_world_state(category, id, "position", position.dump());
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
