#include "executor_tools/AnalyzePerson.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

namespace {

std::string trim(std::string value) {
  const auto is_space = [](unsigned char c) { return std::isspace(c); };

  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(),
                           [&](unsigned char c) { return !is_space(c); }));
  value.erase(std::find_if(value.rbegin(), value.rend(),
                           [&](unsigned char c) { return !is_space(c); })
                  .base(),
              value.end());
  return value;
}

std::string lower(std::string value) {
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::vector<std::string> splitUnderscore(const std::string &value) {
  std::vector<std::string> parts;
  std::stringstream ss(value);
  std::string token;

  while (std::getline(ss, token, '_')) {
    token = trim(token);
    if (!token.empty()) {
      parts.push_back(token);
    }
  }

  return parts;
}

geometry_msgs::msg::PoseStamped
navigationPoseFromPoint(const geometry_msgs::msg::PointStamped &point,
                        double scale) {
  const double px = point.point.x;
  const double py = point.point.y;

  const double yaw = std::atan2(py, px);

  geometry_msgs::msg::PoseStamped nav;
  nav.header = point.header;
  nav.pose.position.x = px * scale;
  nav.pose.position.y = py * scale;
  nav.pose.position.z = 0.0;
  nav.pose.orientation.x = 0.0;
  nav.pose.orientation.y = 0.0;
  nav.pose.orientation.z = std::sin(yaw / 2.0);
  nav.pose.orientation.w = std::cos(yaw / 2.0);

  return nav;
}

struct HeadScanPose {
  const char *label;
  double pan;
  double tilt;
  std::chrono::milliseconds settle;
};

const std::vector<HeadScanPose> &gestureScanPoses() {
  static const std::vector<HeadScanPose> poses = {
      {"center_down", 0.0, -0.60, 1500ms},
      {"left_down", -0.55, -0.55, 900ms},
      {"right_down", 0.55, -0.55, 900ms},
      {"center_mid", 0.0, -0.35, 700ms},
  };
  return poses;
}

std::vector<std::string> gestureCandidates(const std::string &gesture_type) {
  if (gesture_type.empty() || gesture_type == "any" ||
      gesture_type == "unknown") {
    return {"waving", "raising", "pointing"};
  }

  return {gesture_type};
}

std::string spokenGestureName(const std::string &gesture_type) {
  if (gesture_type == "raising" || gesture_type == "raise") {
    return "raising a hand";
  }
  if (gesture_type == "waving" || gesture_type == "wave") {
    return "waving";
  }
  if (gesture_type == "pointing" || gesture_type == "point") {
    return "pointing";
  }
  return gesture_type.empty() ? "unknown" : gesture_type;
}

} // namespace

AnalyzePerson::AnalyzePerson(const rclcpp::Node::SharedPtr &node,
                             const std::shared_ptr<Hardware> &hardware,
                             const std::shared_ptr<Vision> &vision,
                             const std::shared_ptr<Knowledge> &knowledge)
    : node_(node), hardware_(hardware), vision_(vision), knowledge_(knowledge) {
}

Result AnalyzePerson::run(const std::string &argument,
                          const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active()) {
    return {false, "FAILED", "goal_inactive or null"};
  }

  publishFeedback(gh, AnalyzePersonState::READY);

  nlohmann::json json_args;
  try {
    publishFeedback(gh, AnalyzePersonState::PARSING_ARGS);
    if (!argument.empty() && argument.front() == '{') {
      json_args = nlohmann::json::parse(argument);
    } else {
      json_args = parsePersonString(argument);
    }
  } catch (const std::exception &e) {
    publishFeedback(gh, AnalyzePersonState::FAILED, "invalid_json_argument");
    return {false, "FAILED", "invalid_json_argument chet"};
  }

  if (!json_args.contains("action")) {
    publishFeedback(gh, AnalyzePersonState::FAILED, "missing_action_field");
    return {false, "FAILED", "missing_action_field"};
  }

  const std::string action =
      lower(trim(json_args.value("action", std::string{})));
  std::string result_msg;
  std::vector<Vision::YoloDetection> detections;
  bool has_point = false;
  geometry_msgs::msg::PointStamped point;
  bool execution_success = false;
  std::string detected_gesture;

  if (action == "color" || action == "attribute") {
    publishFeedback(gh, AnalyzePersonState::MOVING_HEAD, "center_down");
    hardware_->MoveHead({0.0, -0.60});
    std::this_thread::sleep_for(1500ms);

    publishFeedback(gh, AnalyzePersonState::STABILIZING, "center_down");
    std::this_thread::sleep_for(500ms);

    publishFeedback(gh, AnalyzePersonState::CALLING_SERVICE, action);
    const std::string color =
        lower(trim(json_args.value("color", std::string{})));
    const std::string attribute =
        lower(trim(json_args.value("attribute", std::string{})));
    execution_success = runColor(color, attribute, result_msg, detections,
                                 has_point, point);
  } else if (action == "gesture") {
    const std::string gesture_type =
        lower(trim(json_args.value("gesture_type", std::string{})));
    const std::string arm_side =
        lower(trim(json_args.value("arm_side", std::string{"any"})));
    const auto candidates = gestureCandidates(gesture_type);

    for (const auto &pose : gestureScanPoses()) {
      if (!gh->is_active()) {
        return {false, "FAILED", "goal_cancelled"};
      }

      publishFeedback(gh, AnalyzePersonState::MOVING_HEAD, pose.label);
      hardware_->MoveHead({pose.pan, pose.tilt});
      std::this_thread::sleep_for(pose.settle);

      publishFeedback(gh, AnalyzePersonState::STABILIZING, pose.label);
      std::this_thread::sleep_for(400ms);

      for (const auto &candidate : candidates) {
        publishFeedback(gh, AnalyzePersonState::CALLING_SERVICE,
                        action + ":" + candidate + ":" + pose.label);
        execution_success = runGesture(candidate, arm_side, result_msg,
                                       detections, has_point, point);
        if (execution_success) {
          detected_gesture = spokenGestureName(candidate);
          break;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "[AnalyzePerson] gesture '%s' not found at head pose '%s': %s",
                    candidate.c_str(), pose.label, result_msg.c_str());
      }

      if (execution_success) {
        break;
      }
    }
  } else if (action == "combined" || action == "gesture_color") {
    const std::string gesture_type =
        lower(trim(json_args.value("gesture_type", std::string{})));
    const std::string color =
        lower(trim(json_args.value("color", std::string{})));
    const std::string attribute =
        lower(trim(json_args.value("attribute", std::string{})));
    const std::string arm_side =
        lower(trim(json_args.value("arm_side", std::string{"any"})));
    const auto candidates = gestureCandidates(gesture_type);

    for (const auto &pose : gestureScanPoses()) {
      if (!gh->is_active()) {
        return {false, "FAILED", "goal_cancelled"};
      }

      publishFeedback(gh, AnalyzePersonState::MOVING_HEAD, pose.label);
      hardware_->MoveHead({pose.pan, pose.tilt});
      std::this_thread::sleep_for(pose.settle);

      publishFeedback(gh, AnalyzePersonState::STABILIZING, pose.label);
      std::this_thread::sleep_for(400ms);

      for (const auto &candidate : candidates) {
        publishFeedback(gh, AnalyzePersonState::CALLING_SERVICE,
                        action + ":" + candidate + ":" + pose.label);
        execution_success =
            runCombined(candidate, color, attribute, arm_side, result_msg,
                        detections, has_point, point);
        if (execution_success) {
          detected_gesture = spokenGestureName(candidate);
          break;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "[AnalyzePerson] combined gesture '%s' not found at head pose '%s': %s",
                    candidate.c_str(), pose.label, result_msg.c_str());
      }

      if (execution_success) {
        break;
      }
    }
  } else {
    publishFeedback(gh, AnalyzePersonState::FAILED,
                    "unknown_action: " + action);
    return {false, "FAILED", "unknown_action"};
  }

  if (execution_success) {
    publishFeedback(gh, AnalyzePersonState::UPDATING_KNOWLEDGE);
    if (!updateUserNavigation(detections, has_point, point, result_msg)) {
      publishFeedback(gh, AnalyzePersonState::FAILED, result_msg);
      return {false, "FAILED", result_msg};
    }
    if (!detected_gesture.empty() &&
        !updateGestureAnalytics(detected_gesture, result_msg, result_msg)) {
      publishFeedback(gh, AnalyzePersonState::FAILED, result_msg);
      return {false, "FAILED", result_msg};
    }

    publishFeedback(gh, AnalyzePersonState::SUCCESS, result_msg);
    return {true, "SUCCESS", "person_analyzed:" + result_msg};
  }

  publishFeedback(gh, AnalyzePersonState::FAILED, result_msg);
  return {false, "FAILED",
          result_msg.empty() ? "person_analytics_failed" : result_msg};
}

bool AnalyzePerson::runColor(const std::string &color,
                             const std::string &attribute,
                             std::string &out_msg,
                             std::vector<Vision::YoloDetection>
                                 &out_detections,
                             bool &out_has_point,
                             geometry_msgs::msg::PointStamped &out_point) {
  return vision_->AnalyzePerson("color", color, attribute, "", "any", out_msg,
                                out_detections, out_has_point, out_point);
}

bool AnalyzePerson::runGesture(const std::string &gesture_type,
                               const std::string &arm_side,
                               std::string &out_msg,
                               std::vector<Vision::YoloDetection>
                                   &out_detections,
                               bool &out_has_point,
                               geometry_msgs::msg::PointStamped &out_point) {
  return vision_->AnalyzePerson("gesture", "", "", gesture_type, arm_side,
                                out_msg, out_detections, out_has_point,
                                out_point);
}

bool AnalyzePerson::runCombined(const std::string &gesture_type,
                                const std::string &color,
                                const std::string &attribute,
                                const std::string &arm_side,
                                std::string &out_msg,
                                std::vector<Vision::YoloDetection>
                                    &out_detections,
                                bool &out_has_point,
                                geometry_msgs::msg::PointStamped &out_point) {
  return vision_->AnalyzePerson("combined", color, attribute, gesture_type,
                                arm_side, out_msg, out_detections,
                                out_has_point, out_point);
}

bool AnalyzePerson::updateUserNavigation(
    const std::vector<Vision::YoloDetection> &detections,
    bool has_point,
    const geometry_msgs::msg::PointStamped &point,
    std::string &out_msg) {
  geometry_msgs::msg::PoseStamped nav;

  if (has_point) {
    nav = navigationPoseFromPoint(point, 0.95);
  } else {
    if (detections.empty()) {
      out_msg = "person_pose_failed_no_detection_bbox";
      return false;
    }

    const auto best_detection = std::max_element(
        detections.begin(), detections.end(),
        [](const Vision::YoloDetection &lhs, const Vision::YoloDetection &rhs) {
          return lhs.confidence < rhs.confidence;
        });

    geometry_msgs::msg::PoseStamped pose;
    if (!vision_->DetectionToNavPose(*best_detection, pose, nav)) {
      out_msg = "person_pose_failed_bbox_to_pose";
      return false;
    }
  }

  const auto &np = nav.pose.position;
  const auto &nq = nav.pose.orientation;
  nlohmann::json navigation = {np.x, np.y, np.z, nq.x, nq.y, nq.z, nq.w};

  const std::string frame =
      nav.header.frame_id.empty() ? "base_link" : nav.header.frame_id;

  const bool nav_ok = knowledge_->update_world_state("user", "2", "navigation", navigation.dump());
  //const bool frame_ok = knowledge_->update_world_state("user", "2", "navigation_frame", frame);

  //if (!nav_ok || !frame_ok) {
    //out_msg = "person_pose_failed_update_world_state";
    //return false;
  //}

  RCLCPP_INFO(node_->get_logger(),
              "[AnalyzePerson] user_2 navigation updated frame=%s pose=%s",
              frame.c_str(), navigation.dump().c_str());

  return true;
}

bool AnalyzePerson::updateGestureAnalytics(const std::string &gesture,
                                           const std::string &detail,
                                           std::string &out_msg) {
  const bool target_ok =
      knowledge_->update_world_state("analytics", "gesture", "target", "person");
  const bool gesture_ok =
      knowledge_->update_world_state("analytics", "gesture", "gesture", gesture);
  const bool detail_ok =
      knowledge_->update_world_state("analytics", "gesture", "detail", detail);

  if (!target_ok || !gesture_ok || !detail_ok) {
    out_msg = "gesture_analytics_update_failed";
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[AnalyzePerson] analytics.gesture updated gesture=%s detail=%s",
              gesture.c_str(), detail.c_str());
  return true;
}

void AnalyzePerson::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                                    AnalyzePersonState state,
                                    const std::string &detail) {
  if (!gh->is_active()) {
    return;
  }

  std::string msg = "[analyze_person] " + stateName(state);
  if (!detail.empty()) {
    msg += ": " + detail;
  }

  auto fb = std::make_shared<ExecuteTask::Feedback>();
  fb->feedback.message = msg;
  gh->publish_feedback(fb);

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

std::string AnalyzePerson::stateName(AnalyzePersonState state) {
  switch (state) {
  case AnalyzePersonState::READY:
    return "READY";
  case AnalyzePersonState::MOVING_HEAD:
    return "MOVING_HEAD";
  case AnalyzePersonState::STABILIZING:
    return "STABILIZING";
  case AnalyzePersonState::PARSING_ARGS:
    return "PARSING_ARGS";
  case AnalyzePersonState::CALLING_SERVICE:
    return "CALLING_SERVICE";
  case AnalyzePersonState::UPDATING_KNOWLEDGE:
    return "UPDATING_KNOWLEDGE";
  case AnalyzePersonState::SUCCESS:
    return "SUCCESS";
  case AnalyzePersonState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}

nlohmann::json AnalyzePerson::parsePersonString(const std::string &argument) {
  nlohmann::json j;
  const auto parts = splitUnderscore(lower(trim(argument)));

  if (parts.empty()) {
    return j;
  }

  if ((parts[0] == "color" || parts[0] == "attribute") && parts.size() >= 3) {
    j["action"] = parts[0];
    j["color"] = parts[1];
    j["attribute"] = parts[2];
    return j;
  }

  if (parts[0] == "gesture" && parts.size() >= 2) {
    j["action"] = "gesture";
    j["gesture_type"] = parts[1];
    j["arm_side"] = parts.size() >= 3 ? parts[2] : "any";
    return j;
  }

  if ((parts[0] == "combined" || parts[0] == "gesture_color") &&
      parts.size() >= 4) {
    j["action"] = parts[0];
    j["gesture_type"] = parts[1];
    j["color"] = parts[2];
    j["attribute"] = parts[3];
    j["arm_side"] = parts.size() >= 5 ? parts[4] : "any";
    return j;
  }

  return j;
}
