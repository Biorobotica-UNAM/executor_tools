#include "executor_tools/SaveLoc.hpp"

#include <algorithm>
#include <cctype>
#include <utility>

SaveLoc::SaveLoc(const rclcpp::Node::SharedPtr &node,
                 const std::shared_ptr<Navigation> &navigation,
                 const PoseBuffer &customer_pose_map)
    : node_(node),
      navigation_(navigation),
      customer_pose_map_(customer_pose_map) {}

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
  const bool save_customer_pose = normalize(name) == "customer";

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
    case SaveLocState::READY:
      return "READY";
    case SaveLocState::SAVING:
      return "SAVING";
    case SaveLocState::SUCCESS:
      return "SUCCESS";
    case SaveLocState::FAILED:
      return "FAILED";
    default:
      return "UNKNOWN";
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
