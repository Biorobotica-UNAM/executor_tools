#include "executor_tools/AnalyzePerson.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

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
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
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

} // namespace

AnalyzePerson::AnalyzePerson(const rclcpp::Node::SharedPtr &node,
                             const std::shared_ptr<Hardware> &hardware,
                             const std::shared_ptr<Vision> &vision)
    : node_(node), hardware_(hardware), vision_(vision) {}

Result AnalyzePerson::run(const std::string &argument,
                          const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active()) {
    return {false, "FAILED", "goal_inactive or null"};
  }

  publishFeedback(gh, AnalyzePersonState::READY);

  publishFeedback(gh, AnalyzePersonState::MOVING_HEAD);
  hardware_->MoveHead({0.0, -0.60});
  std::this_thread::sleep_for(1500ms);

  publishFeedback(gh, AnalyzePersonState::STABILIZING);
  std::this_thread::sleep_for(500ms);

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
  bool execution_success = false;

  publishFeedback(gh, AnalyzePersonState::CALLING_SERVICE, action);

  if (action == "color" || action == "attribute") {
    const std::string color =
        lower(trim(json_args.value("color", std::string{})));
    const std::string attribute =
        lower(trim(json_args.value("attribute", std::string{})));
    execution_success = runColor(color, attribute, result_msg);
  } else if (action == "gesture") {
    const std::string gesture_type =
        lower(trim(json_args.value("gesture_type", std::string{})));
    const std::string arm_side =
        lower(trim(json_args.value("arm_side", std::string{"any"})));
    execution_success = runGesture(gesture_type, arm_side, result_msg);
  } else if (action == "combined" || action == "gesture_color") {
    const std::string gesture_type =
        lower(trim(json_args.value("gesture_type", std::string{})));
    const std::string color =
        lower(trim(json_args.value("color", std::string{})));
    const std::string attribute =
        lower(trim(json_args.value("attribute", std::string{})));
    const std::string arm_side =
        lower(trim(json_args.value("arm_side", std::string{"any"})));
    execution_success =
        runCombined(gesture_type, color, attribute, arm_side, result_msg);
  } else {
    publishFeedback(gh, AnalyzePersonState::FAILED,
                    "unknown_action: " + action);
    return {false, "FAILED", "unknown_action"};
  }

  if (execution_success) {
    publishFeedback(gh, AnalyzePersonState::SUCCESS, result_msg);
    return {true, "SUCCESS", result_msg};
  }

  publishFeedback(gh, AnalyzePersonState::FAILED, result_msg);
  return {false, "FAILED",
          result_msg.empty() ? "person_analytics_failed" : result_msg};
}

bool AnalyzePerson::runColor(const std::string &color,
                             const std::string &attribute,
                             std::string &out_msg) {
  std::vector<Vision::YoloDetection> detections;
  return vision_->AnalyzePerson("color", color, attribute, "", "any", out_msg,
                                detections);
}

bool AnalyzePerson::runGesture(const std::string &gesture_type,
                               const std::string &arm_side,
                               std::string &out_msg) {
  std::vector<Vision::YoloDetection> detections;
  return vision_->AnalyzePerson("gesture", "", "", gesture_type, arm_side,
                                out_msg, detections);
}

bool AnalyzePerson::runCombined(const std::string &gesture_type,
                                const std::string &color,
                                const std::string &attribute,
                                const std::string &arm_side,
                                std::string &out_msg) {
  std::vector<Vision::YoloDetection> detections;
  return vision_->AnalyzePerson("combined", color, attribute, gesture_type,
                                arm_side, out_msg, detections);
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
  case AnalyzePersonState::SUCCESS:
    return "SUCCESS";
  case AnalyzePersonState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}

nlohmann::json AnalyzePerson::parsePersonString(
    const std::string &argument) {
  nlohmann::json j;
  const auto parts = splitUnderscore(lower(trim(argument)));

  if (parts.empty()) {
    return j;
  }

  if ((parts[0] == "color" || parts[0] == "attribute") &&
      parts.size() >= 3) {
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
