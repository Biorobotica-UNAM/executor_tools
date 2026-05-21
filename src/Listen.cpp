/**
 * @brief FSM implementation for the listen pipeline.
 *
 * Each private method is responsible for exactly one FSM transition.
 * run() owns the sequence and early-exits on any failure, keeping
 * cyclomatic complexity low and the happy path readable.
 *
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include "executor_tools/Listen.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================

Listen::Listen(const rclcpp::Node::SharedPtr &node,
               const std::shared_ptr<HRI> &hri,
               const std::shared_ptr<Planning> &planning,
               const std::shared_ptr<Knowledge> &knowledge)
    : node_(node), hri_(hri), planning_(planning), knowledge_(knowledge) {}

// =========================================================
// run
// =========================================================

Result Listen::run(const std::string &argument,
                   const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active())
    return {false, "FAILED", "goal_handle_inactive"};

  // ── READY ──────────────────────────────────────────────
  publishFeedback(gh, ListenState::READY, argument);

  std::string category;
  std::string index;

  if (!parseArgument(argument, category, index)) {
    publishFeedback(gh, ListenState::FAILED, "invalid_argument: " + argument);
    return {false, "FAILED", "invalid_argument: " + argument};
  }

  // ── LISTENING ──────────────────────────────────────────
  publishFeedback(gh, ListenState::LISTENING);

  const std::string speech = hri_->Listen("", 20s);

  if (speech.empty()) {
    publishFeedback(gh, ListenState::FAILED, "listen_failed");
    return {false, "FAILED", "listen_failed"};
  }

  RCLCPP_INFO(node_->get_logger(), "Heard: %s", speech.c_str());

  // ── PARSING (with retry) ────────────────────────────────
  publishFeedback(gh, ListenState::PARSING, speech);

  const std::string entity = extractEntityWithRetry(speech, category, gh);

  if (entity.empty()) {
    publishFeedback(gh, ListenState::FAILED,
                    "entity_not_found: category=" + category);
    return {false, "FAILED", "entity_not_found: category=" + category};
  }

  RCLCPP_INFO(node_->get_logger(), "Extracted entity [%s] = '%s'",
              category.c_str(), entity.c_str());

  // ── UPDATING ───────────────────────────────────────────
  publishFeedback(gh, ListenState::UPDATING,
                  "people[" + index + "]." + category + "=" + entity);

  const nlohmann::json payload = buildUpdatePayload(index, category, entity);

  const bool stored = knowledge_->update_world_state(payload);

  if (!stored) {
    publishFeedback(gh, ListenState::FAILED, "update_world_state_failed");
    return {false, "FAILED", "update_world_state_failed"};
  }

  // ── SUCCESS ────────────────────────────────────────────
  const std::string summary =
      "people[" + index + "]." + category + "=" + entity;

  publishFeedback(gh, ListenState::SUCCESS, summary);
  return {true, "SUCCESS", summary};
}

// =========================================================
// extractEntityWithRetry
// =========================================================

std::string
Listen::extractEntityWithRetry(const std::string &speech,
                               const std::string &category,
                               const std::shared_ptr<GoalHandle> &gh) const {
  constexpr int kMaxRetries = 3;
  constexpr auto kRetryDelay = 500ms;

  for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
    const std::string entity = planning_->extractEntity(speech, category);

    if (!entity.empty())
      return entity;

    if (attempt < kMaxRetries) {
      RCLCPP_WARN(
          node_->get_logger(),
          "extractEntity attempt %d/%d returned empty — retrying in %ld ms.",
          attempt, kMaxRetries,
          std::chrono::duration_cast<std::chrono::milliseconds>(kRetryDelay)
              .count());
      std::this_thread::sleep_for(kRetryDelay);
    }
  }

  return "";
}

// =========================================================
// parseArgument
// =========================================================

bool Listen::parseArgument(const std::string &argument,
                           std::string &out_category,
                           std::string &out_index) const {
  const std::size_t sep = argument.rfind('_');

  if (sep == std::string::npos || sep == 0 || sep + 1 == argument.size()) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Listen: argument '%s' must follow the pattern <category>_<index>.",
        argument.c_str());
    return false;
  }

  out_category = argument.substr(0, sep);
  out_index = argument.substr(sep + 1);
  return true;
}

// =========================================================
// buildUpdatePayload
// =========================================================

nlohmann::json Listen::buildUpdatePayload(const std::string &index,
                                          const std::string &category,
                                          const std::string &value) const {
  nlohmann::json person;
  person["number"] = std::stoi(index);
  person[category] = value;

  return {{"person", person}};
}

// =========================================================
// publishFeedback
// =========================================================

void Listen::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                             ListenState state, const std::string &detail) {
  if (!gh->is_active())
    return;

  std::string msg = "[listen] " + stateName(state);
  if (!detail.empty())
    msg += ": " + detail;

  auto fb = std::make_shared<ExecuteTask::Feedback>();
  fb->feedback.message = msg;
  gh->publish_feedback(fb);

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================

std::string Listen::stateName(ListenState s) {
  switch (s) {
  case ListenState::READY:
    return "READY";
  case ListenState::LISTENING:
    return "LISTENING";
  case ListenState::PARSING:
    return "PARSING";
  case ListenState::UPDATING:
    return "UPDATING";
  case ListenState::SUCCESS:
    return "SUCCESS";
  case ListenState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}
