/**
 * @brief FSM that retrieves a phrase from the knowledge bank and speaks it.
 *
 * Flow: READY → LOADING → SPEAKING → SUCCESS | FAILED
 *
 * Design patterns applied:
 *   Facade          (GoF Structural) — delegates to Knowledge and HRI.
 *   Template Method (GoF Behavioral) — run() defines the invariant FSM
 *                                      sequence; each private step supplies
 *                                      only what varies.
 * Principles applied:
 *   SRP  — one class, one responsibility: orchestrate the say pipeline.
 *   DIP  — depends on HRI / Knowledge abstractions, not on rclcpp internals.
 *
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include "executor_tools/Say.hpp"

// =========================================================
// Constructor
// =========================================================
Say::Say(const rclcpp::Node::SharedPtr&    node,
         const std::shared_ptr<HRI>&       hri,
         const std::shared_ptr<Knowledge>& knowledge)
: node_(node), hri_(hri), knowledge_(knowledge)
{}

// =========================================================
// run
// =========================================================
Result Say::run(const std::string&                 phrase_key,
                const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, SayState::READY, phrase_key);

    // ── LOADING ───────────────────────────────────────────
    // Knowledge::get_phrase() busca la key en phrases/,
    // selecciona una frase aleatoria y resuelve {placeholders}
    // contra el world state. Devuelve la cadena lista para TTS.
    publishFeedback(gh, SayState::LOADING);

    const std::string utterance = knowledge_->get_phrase(phrase_key, "");

    // --- EFECTO FALLBACK ---
    if (utterance.empty()) {
        // Si el Knowledge no encontró la llave, asumimos que 'phrase_key' 
        // es el texto directo que se quiere mandar a reproducir.
        RCLCPP_INFO(node_->get_logger(), 
                    "Say: Key '%s' not found in YAML. Using raw argument as fallback.", 
                    phrase_key.c_str());
        utterance = phrase_key; 
    }

    RCLCPP_INFO(node_->get_logger(), "Say: '%s'", utterance.c_str());

    // ── SPEAKING ──────────────────────────────────────────
    publishFeedback(gh, SayState::SPEAKING, utterance);

    const bool ok = hri_->Say(utterance);

    // ── SUCCESS / FAILED ──────────────────────────────────
    if (ok) {
        publishFeedback(gh, SayState::SUCCESS, utterance);
        return {true, "SUCCESS", "uttered: " + utterance};
    }

    publishFeedback(gh, SayState::FAILED, "tts_failed");
    return {false, "FAILED", "tts_failed: " + utterance};
}

// =========================================================
// publishFeedback
// =========================================================
void Say::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                          SayState                           state,
                          const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[say] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string Say::stateName(SayState s)
{
    switch (s) {
        case SayState::READY:    return "READY";
        case SayState::LOADING:  return "LOADING";
        case SayState::SPEAKING: return "SPEAKING";
        case SayState::SUCCESS:  return "SUCCESS";
        case SayState::FAILED:   return "FAILED";
        default:                 return "UNKNOWN";
    }
}
