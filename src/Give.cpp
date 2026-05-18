#include "executor_tools/Give.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Give::Give(const rclcpp::Node::SharedPtr& node,
           const std::shared_ptr<Hardware>& hardware,
           const std::shared_ptr<HRI>& hri)
: node_(node), hardware_(hardware), hri_(hri)
{}

// =========================================================
// run — El argumento 'object_name' es el objeto dinámico
// =========================================================
Result Give::run(const std::string& object_name,
                 const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    std::string item = object_name.empty() ? "item" : object_name;

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, GiveState::READY, item);

    // 1. Extender el brazo inmediatamente 
    publishFeedback(gh, GiveState::MOVING_ARM, "Extending arm to give " + item);
    hardware_->MoveBase(0.0, 0.0, 0.0);
    bool arm_ok = hardware_->MoveLeftArm(
        {ARM_POINTING[0], ARM_POINTING[1], ARM_POINTING[2],
         ARM_POINTING[3], ARM_POINTING[4], ARM_POINTING[5]});

    if (!arm_ok) {
        publishFeedback(gh, GiveState::FAILED, "arm_failed_extension");
        return {false, "FAILED", "arm_extension_failed"};
    }

    // 2. Decir frase usando el nombre del objeto recibido
    publishFeedback(gh, GiveState::SPEAKING, "Requesting retrieval of: " + item);
    std::string phrase = "Please take the " + item + ". I will open my hand in 5. 4. 3. 2. 1.";
    hri_->Say(phrase);

    // 3. Abrir la mano 
    publishFeedback(gh, GiveState::MOVING_ARM, "Opening gripper");
    // hardware_->OpenGripper();

    // 4. Esperar 2 segundos mientras la persona retira el objeto
    publishFeedback(gh, GiveState::WAITING, "Waiting 2 seconds");
    std::this_thread::sleep_for(2s);

    // 5. Cerrar mano y regresar el brazo a la posición de reposo
    publishFeedback(gh, GiveState::MOVING_ARM, "Resetting arm to resting position");
    // hardware_->CloseGripper();
    std::this_thread::sleep_for(500ms);

    bool reset_ok = hardware_->MoveLeftArm(
        {ARM_RESTING[0], ARM_RESTING[1], ARM_RESTING[2],
         ARM_RESTING[3], ARM_RESTING[4], ARM_RESTING[5]});

    if (reset_ok) {
        publishFeedback(gh, GiveState::SUCCESS, "give_completed_for_" + item);
        return {true, "SUCCESS", item + "_delivered_successfully"};
    } else {
        publishFeedback(gh, GiveState::FAILED, "arm_failed_reset");
        return {false, "FAILED", "arm_reset_failed_after_give"};
    }
}

// =========================================================
// publishFeedback — Formato anidado fb->feedback.message
// =========================================================
void Give::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                            GiveState                          state,
                            const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[give] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg; // TOTALMENTE ALINEADO A TU .ACTION REAL
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string GiveStateName(GiveState s)
{
    switch (s) {
        case GiveState::READY:      return "READY";
        case GiveState::MOVING_ARM: return "MOVING_ARM";
        case GiveState::SPEAKING:   return "SPEAKING";
        case GiveState::WAITING:    return "WAITING";
        case GiveState::SUCCESS:    return "SUCCESS";
        case GiveState::FAILED:     return "FAILED";
        default:                    return "UNKNOWN";
    }
}
