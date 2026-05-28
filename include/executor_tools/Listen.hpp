#pragma once

/**
 * @brief FSM that captures speech, extracts a named entity, and
 *        (optionally) persists it into the world-state knowledge base.
 *
 * Flow completo  (argumento tipo "guest_name_1"):
 *   READY → LISTENING → PARSING → UPDATING → SUCCESS | FAILED
 *
 * Flujo simple (argumento tipo "follow" o "confirm", sin separador '_'):
 *   READY → LISTENING → PARSING → SUCCESS | FAILED
 *   El paso UPDATING se omite; la entidad extraída se devuelve en el
 *   campo detail del Result sin escribir en el world state.
 *
 * Design patterns applied:
 *   Facade          (GoF Structural) — delegates to HRI, Planning, Knowledge.
 *   Template Method (GoF Behavioral) — run() defines the invariant FSM
 *                                      sequence; each private step supplies
 *                                      only what varies.
 * Principles applied:
 *   SRP  — one class, one responsibility: orchestrate the listen pipeline.
 *   DIP  — depends on HRI / Planning / Knowledge abstractions, not on
 *          rclcpp service internals.
 *   OCP  — new entity categories can be added without modifying this class.
 *
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hri_tools/HRI.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "planning_tools/Planning.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <string>

// =========================================================
// FSM states
// =========================================================
enum class ListenState { READY, LISTENING, PARSING, UPDATING, SUCCESS, FAILED };

// =========================================================
// Listen modes
// =========================================================
/**
 * FULL   — argumento con patrón "<category>_<attribute>_<index>"
 *          (ej. "guest_name_1"). Ejecuta los tres pasos: escuchar,
 *          extraer y actualizar el world state.
 *
 * SIMPLE — argumento sin separadores (ej. "follow", "confirm").
 *          Ejecuta solo escuchar y extraer; omite la actualización
 *          del world state.
 */
enum class ListenMode { FULL, SIMPLE };

class Listen {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Listen(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<HRI> &hri,
         const std::shared_ptr<Planning> &planning,
         const std::shared_ptr<Knowledge> &knowledge);

  /**
   * Execute the full listen pipeline synchronously.
   *
   * El modo de operación se determina automáticamente a partir del argumento:
   *
   * - Modo FULL   : argumento sigue el patrón "<category>_<attribute>_<index>"
   *                 (ej. "guest_name_1").  Pasos: escuchar → extraer → guardar.
   *                 Retorna Result{true, "SUCCESS", "people[N].<attr>=<val>"}.
   *
   * - Modo SIMPLE : argumento es una palabra simple sin '_'
   *                 (ej. "follow", "confirm").  Pasos: escuchar → extraer.
   *                 No toca el world state.
   *                 Retorna Result{true, "SUCCESS", "<argument>=<entity>"}.
   *
   * @param argument  Cadena que codifica la operación deseada.
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
  std::vector<std::string>
  extractEntityWithRetry(const std::string &speech, const std::string &category,
                         const std::shared_ptr<GoalHandle> &gh) const;

  /**
   * Intenta parsear el argumento con patrón "<category>_<attribute>_<index>".
   *
   * Retorna ListenMode::FULL y rellena out_* si el patrón se cumple.
   * Retorna ListenMode::SIMPLE si el argumento no contiene '_', en cuyo
   * caso out_category toma el valor completo del argumento y out_index /
   * out_attribute quedan vacíos.
   * Retorna ListenMode::FULL con return=false si el argumento tiene '_'
   * pero no cumple el patrón completo (error de formato).
   *
   * @param argument      Cadena de entrada.
   * @param out_category  Categoría de la entidad (parte inicial).
   * @param out_index     Índice de slot de persona (parte final).
   * @param out_attribute Atributo a extraer (parte central).
   * @param out_mode      Modo detectado (FULL o SIMPLE).
   * @return true si el argumento es válido (FULL o SIMPLE), false si
   *         tiene '_' pero no satisface el patrón esperado.
   */
  bool parseArgument(const std::string &argument, std::string &out_category,
                     std::string &out_index, std::string &out_attribute,
                     ListenMode &out_mode) const;

  /**
   * Build the JSON payload expected by Knowledge::update_world_state().
   *
   * Schema:
   * {
   *   "category": "<category>",
   *   "people": {
   *     "number": <index as int>,
   *     "<attribute>": "<value>"
   *   }
   * }
   */
  nlohmann::json buildUpdatePayload(const std::string &index,
                                    const std::string &category,
                                    const std::string &entity_type,
                                    const std::string &attribute,
                                    const nlohmann::json &value) const;

  void publishFeedback(const std::shared_ptr<GoalHandle> &gh, ListenState state,
                       const std::string &detail = "");

  static std::string stateName(ListenState s);

  // ── collaborators ──────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<HRI> hri_;
  std::shared_ptr<Planning> planning_;
  std::shared_ptr<Knowledge> knowledge_;
};
