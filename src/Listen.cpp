/**
 * @brief FSM implementation for the listen pipeline.
 *
 * Soporta dos modos de operación determinados automáticamente por el
 * formato del argumento recibido en run():
 *
 *   FULL   — "<category>_<attribute>_<index>"  (ej. "guest_name_1")
 *            Pasos: READY → LISTENING → PARSING → UPDATING → SUCCESS | FAILED
 *
 *   SIMPLE — palabra simple sin '_'  (ej. "follow", "confirm")
 *            Pasos: READY → LISTENING → PARSING → SUCCESS | FAILED
 *            El world state no se modifica; la entidad extraída se
 *            devuelve en el campo detail del Result.
 *
 * Cada método privado es responsable de exactamente una transición FSM.
 * run() posee la secuencia y hace early-exit ante cualquier fallo,
 * manteniendo baja la complejidad ciclomática y legible el happy path.
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
  std::string attribute;
  ListenMode mode;

  if (!parseArgument(argument, category, index, attribute, mode)) {
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

  const std::string entity = extractEntityWithRetry(speech, attribute, gh);

  if (entity.empty()) {
    publishFeedback(gh, ListenState::FAILED,
                    "entity_not_found: category=" + attribute);
    return {false, "FAILED", "entity_not_found: category=" + attribute};
  }

  RCLCPP_INFO(node_->get_logger(), "Extracted entity [%s] = '%s'",
              attribute.c_str(), entity.c_str());

  // ── SIMPLE mode: skip UPDATING ─────────────────────────
  if (mode == ListenMode::SIMPLE) {
    const std::string summary = argument + "=" + entity;
    publishFeedback(gh, ListenState::SUCCESS, summary);
    return {true, "SUCCESS", summary};
  }

  // ── UPDATING (FULL mode only) ──────────────────────────
  publishFeedback(gh, ListenState::UPDATING,
                  "people[" + index + "]." + attribute + "=" + entity);

  const nlohmann::json payload =
      buildUpdatePayload(index, category, "person", attribute, entity);

  const bool stored = knowledge_->update_world_state(payload);

  if (!stored) {
    publishFeedback(gh, ListenState::FAILED, "update_world_state_failed");
    return {false, "FAILED", "update_world_state_failed"};
  }

  // ── SUCCESS ────────────────────────────────────────────
  const std::string summary =
      "people[" + index + "]." + attribute + "=" + entity;

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
                           std::string &out_category, std::string &out_index,
                           std::string &out_attribute,
                           ListenMode &out_mode) const {

  // ── Modo SIMPLE: el argumento no contiene '_' ──────────
  if (argument.find('_') == std::string::npos) {
    if (argument.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Listen: argument is empty.");
      return false;
    }
    out_category = argument;
    out_attribute = argument; // se usa como categoría en extractEntity
    out_index = "";
    out_mode = ListenMode::SIMPLE;
    return true;
  }

  // ── Modo FULL: patrón esperado "<category>_<attribute>_<index>" ──
  const std::size_t last_sep = argument.rfind('_');

  if (last_sep == std::string::npos || last_sep == 0 ||
      last_sep + 1 == argument.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Listen: argument '%s' must follow the pattern "
                 "<category>_<attribute>_<index>.",
                 argument.c_str());
    return false;
  }

  const std::string prefix = argument.substr(0, last_sep);
  const std::size_t second_sep = prefix.rfind('_');

  if (second_sep == std::string::npos || second_sep == 0 ||
      second_sep + 1 == prefix.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Listen: argument '%s' must follow the pattern "
                 "<category>_<attribute>_<index>.",
                 argument.c_str());
    return false;
  }

  out_category = prefix.substr(0, second_sep);
  out_attribute = prefix.substr(second_sep + 1);
  out_index = argument.substr(last_sep + 1);
  out_mode = ListenMode::FULL;

  return true;
}

// =========================================================
// buildUpdatePayload
// =========================================================

nlohmann::json Listen::buildUpdatePayload(const std::string &index,
                                          const std::string &category,
                                          const std::string &entity_type,
                                          const std::string &attribute,
                                          const nlohmann::json &value) const {

  nlohmann::json entity;
  entity["number"] = std::stoi(index);
  entity[attribute] = value;

  nlohmann::json payload;
  payload["category"] = category;
  payload[entity_type] = entity;

  return payload;
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
