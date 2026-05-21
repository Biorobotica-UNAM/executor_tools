#pragma once

/**
 * @brief FSM that captures speech, extracts a named entity, and
 *        persists it into the world-state knowledge base.
 *
 * Flow: READY → LISTENING → PARSING → UPDATING → SUCCESS | FAILED
 *
 * Design patterns applied:
 *   Facade          (GoF Structural) — delegates to HRI, Planning, KNOWLEDGE.
 *   Template Method (GoF Behavioral) — run() defines the invariant FSM
 *                                      sequence; each private step supplies
 *                                      only what varies.
 * Principles applied:
 *   SRP  — one class, one responsibility: orchestrate the listen pipeline.
 *   DIP  — depends on HRI / Planning / KNOWLEDGE abstractions, not on
 *          rclcpp service internals.
 *   OCP  — new entity categories can be added without modifying this class.
 *
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hri_tools/HRI.hpp"
#include "knowledge_tools/KNOWLEDGE.hpp"
#include "planning_tools/Planning.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <memory>
#include <string>

// =========================================================
// FSM states
// =========================================================
enum class ListenState { READY, LISTENING, PARSING, UPDATING, SUCCESS, FAILED };

class Listen {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Listen(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<HRI> &hri,
         const std::shared_ptr<Planning> &planning,
         const std::shared_ptr<KNOWLEDGE> &knowledge);

  /**
   * Execute the full listen pipeline synchronously.
   *
   * @param argument  Encodes "<category>_<index>", e.g. "name_1" or "name_2".
   *                  - category : entity category forwarded to
   * Planning::extractEntity().
   *                  - index    : person slot number stored in the world state.
   * @param gh        Active action goal handle used for feedback publishing.
   * @return Result{true, "SUCCESS", …} on full success, Result{false, …}
   * otherwise.
   */
  Result run(const std::string &argument,
             const std::shared_ptr<GoalHandle> &gh);

private:
  // ── helpers ────────────────────────────────────────────────────────────────

  /**
   * Retry wrapper around Planning::extractEntity().
   *
   * The ROS2 service client occasionally returns empty on consecutive
   * calls due to executor scheduling latency. Retrying with a short
   * backoff resolves the issue without modifying the service contract.
   *
   * @param speech    Full recognised speech string.
   * @param category  Entity category (e.g. "name").
   * @param gh        Goal handle — used only to check cancellation between
   * retries.
   * @return Extracted entity string, or empty on exhausted retries.
   */
  std::string
  extractEntityWithRetry(const std::string &speech, const std::string &category,
                         const std::shared_ptr<GoalHandle> &gh) const;

  /**
   * Split "name_1" → {"name", "1"}.
   * Returns false if the argument does not match the expected pattern.
   */
  bool parseArgument(const std::string &argument, std::string &out_category,
                     std::string &out_index) const;

  /**
   * Build the JSON payload expected by KNOWLEDGE::update_world_state().
   *
   * Schema:
   * {
   *   "person": {
   *     "number": <index as int>,
   *     "<category>": "<value>"
   *   }
   * }
   */
  nlohmann::json buildUpdatePayload(const std::string &index,
                                    const std::string &category,
                                    const std::string &value) const;

  void publishFeedback(const std::shared_ptr<GoalHandle> &gh, ListenState state,
                       const std::string &detail = "");

  static std::string stateName(ListenState s);

  // ── collaborators ──────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<HRI> hri_;
  std::shared_ptr<Planning> planning_;
  std::shared_ptr<KNOWLEDGE> knowledge_;
};
