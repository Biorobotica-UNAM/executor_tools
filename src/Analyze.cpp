#include "executor_tools/Analyze.hpp"
#include <chrono>
#include <complex>
#include <cstddef>
#include <string>
#include <thread>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
AnalyzeObjects::AnalyzeObjects(const rclcpp::Node::SharedPtr &node,
                               const std::shared_ptr<Hardware> &hardware,
                               const std::shared_ptr<Vision> &vision,
                               const std::shared_ptr<Knowledge> &knowledge)
    : node_(node), hardware_(hardware), vision_(vision), knowledge_(knowledge) {
}

// ─────────────────────────────────────────────────────────────
//  run
// ─────────────────────────────────────────────────────────────

Result AnalyzeObjects::run(const std::string &argument,
                           const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active())
    return {false, "FAILED", "goal_inactive"};

  publishFeedback(gh, AnalyzeObjectsState::READY);

  // ── Posicionar cabeza ──────────────────────────────
  publishFeedback(gh, AnalyzeObjectsState::MOVING_HEAD);
  hardware_->MoveHead({0.0, -0.60});
  std::this_thread::sleep_for(1500ms);

  publishFeedback(gh, AnalyzeObjectsState::STABILIZING);
  std::this_thread::sleep_for(500ms);
  // ──────────────────────────────────────────────────

  nlohmann::json json_args;
  try {
    publishFeedback(gh, AnalyzeObjectsState::PARSING_ARGS);
    if (!argument.empty() && argument.front() == '{') {
      json_args = nlohmann::json::parse(argument);
    } else {
      json_args = parseObjectString(argument);
    }
  } catch (const std::exception &e) {
    publishFeedback(gh, AnalyzeObjectsState::FAILED, "invalid_json_argument");
    return {false, "FAILED", "invalid_json_argument"};
  }

  if (!json_args.contains("action")) {
    publishFeedback(gh, AnalyzeObjectsState::FAILED, "missing_action_field");
    return {false, "FAILED", "missing_action_field"};
  }

  std::string action = json_args["action"].get<std::string>();
  std::string result_msg = "";
  bool execution_success = false;

  publishFeedback(gh, AnalyzeObjectsState::CALLING_SERVICE, action);

  // 1. ACCIÓN: COUNT
  if (action == "count") {
    std::string target_class = json_args.value("target_class", "");
    bool reset_count = json_args.value("reset_count", false);
    execution_success = runCount(target_class, reset_count, result_msg);
  }
  // 2. ACCIÓN: SIZE (El más grande / pequeño / extremos)
  else if (action == "size") {
    std::string target_class = json_args.value("target_class", "");
    std::string filter_mode =
        json_args.value("filter_mode", "big"); // big, small, mixed
    int32_t limit = json_args.value("count_limit", 1);
    execution_success = runSize(target_class, filter_mode, limit, result_msg);
  }
  // 3. ACCIÓN: COLOR (Dame el objeto de color X)
  else if (action == "color") {
    std::string target_class = json_args.value("target_class", "");
    std::string color = json_args.value("color", "");
    execution_success = runColor(target_class, color, result_msg);
  }
  // 4. ACCIÓN: COMPARE (Cuál categoría tiene más objetos)
  else if (action == "compare") {
    std::vector<std::string> compare_classes =
        json_args.value("compare_classes", std::vector<std::string>{});
    execution_success = runCompare(compare_classes, result_msg);
  }
  // ACCIÓN DESCONOCIDA
  else {
    publishFeedback(gh, AnalyzeObjectsState::FAILED,
                    "unknown_action: " + action);
    return {false, "FAILED", "unknown_action"};
  }

  // Evaluar resultado y responder
  if (execution_success) {
    publishFeedback(gh, AnalyzeObjectsState::SUCCESS, result_msg);
    return {true, "SUCCESS", result_msg};
  } else {
    publishFeedback(gh, AnalyzeObjectsState::FAILED,
                    "service_failed_or_no_objects");
    return {false, "FAILED", "analytics_failed"};
  }
}

// ─────────────────────────────────────────────────────────────
//  Manejadores de acciones específicas
// ─────────────────────────────────────────────────────────────

bool AnalyzeObjects::runCount(const std::string &target_class, bool reset_count,
                              std::string &out_msg) {
  int32_t current_count = 0, class_count = 0, total_cumulative = 0;

  bool ok = vision_->AnalyzeCount(target_class, reset_count, current_count,
                                  class_count, total_cumulative);
  if (ok) {
    knowledge_->update_world_state("analytics", "count", "target_class",
                                   target_class);
    knowledge_->update_world_state("analytics", "count", "class_count",
                                   std::to_string(class_count));
    knowledge_->update_world_state("analytics", "count", "frame_count",
                                   std::to_string(current_count));
    knowledge_->update_world_state("analytics", "count", "cumulative",
                                   std::to_string(total_cumulative));

    out_msg = "counted_" + std::to_string(class_count) + "_" + target_class +
              "_total_frame_" + std::to_string(current_count) + "_cumulative_" +
              std::to_string(total_cumulative);
  }
  return ok;
}

bool AnalyzeObjects::runSize(const std::string &target_class,
                             const std::string &filter_mode, int32_t limit,
                             std::string &out_msg) {
  Vision::YoloDetection best_match;
  std::vector<Vision::YoloDetection> detections;

  bool ok = vision_->AnalyzeSize(target_class, filter_mode, limit, best_match,
                                 detections);
  if (ok) {
    knowledge_->update_world_state("analytics", "size", "target_class",
                                   target_class);
    knowledge_->update_world_state("analytics", "size", "filter_mode",
                                   filter_mode);
    knowledge_->update_world_state("analytics", "size", "best_match",
                                   best_match.class_name);
    knowledge_->update_world_state("analytics", "size", "confidence",
                                   std::to_string(best_match.confidence));

    out_msg = "size_filter_success_best_match_" + best_match.class_name;
  }
  return ok;
}

bool AnalyzeObjects::runColor(const std::string &target_class,
                              const std::string &color, std::string &out_msg) {
  Vision::YoloDetection best_match;

  bool ok = vision_->AnalyzeColor(target_class, color, best_match);
  if (ok) {
    knowledge_->update_world_state("analytics", "color", "target_class",
                                   target_class);
    knowledge_->update_world_state("analytics", "color", "color", color);
    knowledge_->update_world_state("analytics", "color", "best_match",
                                   best_match.class_name);

    out_msg = "color_match_found_" + best_match.class_name;
  }
  return ok;
}

bool AnalyzeObjects::runCompare(const std::vector<std::string> &classes,
                                std::string &out_msg) {
  std::string winner_class;

  bool ok = vision_->AnalyzeCompare(classes, winner_class);
  if (ok) {

    std::string clases_str = "";
    for (size_t i = 0; i < classes.size(); i++) {
      clases_str += classes[i] + (i + 1 < classes.size() ? "," : "");
    }

    knowledge_->update_world_state("analytics", "compare", "classes",
                                   clases_str);
    knowledge_->update_world_state("analytics", "compare", "winner",
                                   winner_class);

    out_msg = "compare_winner_is_" + winner_class;
  }
  return ok;
}

// ─────────────────────────────────────────────────────────────
//  publishFeedback
// ─────────────────────────────────────────────────────────────
void AnalyzeObjects::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                                     AnalyzeObjectsState state,
                                     const std::string &detail) {
  if (!gh->is_active())
    return;

  std::string msg = "[analyze_objects] " + stateName(state);
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
std::string AnalyzeObjects::stateName(AnalyzeObjectsState s) {
  switch (s) {
  case AnalyzeObjectsState::READY:
    return "READY";
  case AnalyzeObjectsState::MOVING_HEAD:
    return "MOVING_HEAD";
  case AnalyzeObjectsState::STABILIZING:
    return "STABILIZING";
  case AnalyzeObjectsState::PARSING_ARGS:
    return "PARSING_ARGS";
  case AnalyzeObjectsState::CALLING_SERVICE:
    return "CALLING_SERVICE";
  case AnalyzeObjectsState::UPDATING_KNOWLEDGE:
    return "UPDATING_KNOWLEDGE";
  case AnalyzeObjectsState::SUCCESS:
    return "SUCCESS";
  case AnalyzeObjectsState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}

// ─────────────────────────────────────────────────────────────
//  parseObjectString
// ─────────────────────────────────────────────────────────────
nlohmann::json AnalyzeObjects::parseObjectString(const std::string &argument) {
  nlohmann::json j;

  if (argument.rfind("count_", 0) == 0) {
    j["action"] = "count";
    j["target_class"] = argument.substr(6);
    j["reset_count"] = false;

    return j;
  }

  if (argument.rfind("biggest_", 0) == 0) {
    j["action"] = "size";
    j["filter_mode"] = "big";
    j["count_limit"] = 1;
    j["target_class"] = argument.substr(8);

    return j;
  }

  if (argument.rfind("smallest_", 0) == 0) {
    j["action"] = "size";
    j["filter_mode"] = "small";
    j["count_limit"] = 1;
    j["target_class"] = argument.substr(9);

    return j;
  }

  // bare class name from CLIPS (e.g. "snacks") → default to count
  if (!argument.empty()) {
    j["action"] = "count";
    j["target_class"] = argument;
    j["reset_count"] = false;
    return j;
  }

  return j;
}
