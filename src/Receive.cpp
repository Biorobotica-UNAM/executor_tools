#include "executor_tools/Receive.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Receive::Receive(const rclcpp::Node::SharedPtr& node,
                 const std::shared_ptr<Hardware>& hardware,
                 const std::shared_ptr<HRI>& hri)
: node_(node), hardware_(hardware), hri_(hri)
{}

// =========================================================
// run — El argumento 'object_name' es el objeto dinámico
// =========================================================
Result Receive::run(const std::string& object_name,
                    const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    std::string item = object_name.empty() ? "item" : object_name;

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, ReceiveState::READY, item);

    // 1. Extender el brazo inmediatamente 
    publishFeedback(gh, ReceiveState::MOVING_ARM, "Extending arm to receive " + item);
    bool arm_ok = hardware_->MoveLeftArm(
        {ARM_POINTING[0], ARM_POINTING[1], ARM_POINTING[2],
         ARM_POINTING[3], ARM_POINTING[4], ARM_POINTING[5]});

    if (!arm_ok) {
        publishFeedback(gh, ReceiveState::FAILED, "arm_failed_extension");
        return {false, "FAILED", "arm_extension_failed"};
    }

    // 2. Abrir el Gripper 
    publishFeedback(gh, ReceiveState::MOVING_GRIPPER, "Opening gripper to receive " + item);
    bool gripper_ok = hardware_->MoveLeftGripper("open");

    if (!gripper_ok) {
        publishFeedback(gh, ReceiveState::FAILED, "gripper_open_failed");
        return {false, "FAILED", "gripper_open_failed"};
    }

    std::this_thread::sleep_for(200ms); 

    // 3. Hablar 
    publishFeedback(gh, ReceiveState::SPEAKING, "Requesting placement of: " + item);
    std::string phrase = "Please place the " + item + " in my hand. I will close my hand in 5. 4. 3. 2. 1.";
    hri_->Say(phrase);

    // 4. Cerrar la mano 
    publishFeedback(gh, ReceiveState::MOVING_GRIPPER, "Closing gripper on " + item);
    gripper_ok = hardware_->MoveLeftGripper("close");

    if (!gripper_ok) {
        publishFeedback(gh, ReceiveState::FAILED, "gripper_close_failed");
        return {false, "FAILED", "gripper_close_failed"};
    }
    std::this_thread::sleep_for(1s); 

    // 5. Regresar el brazo a la posición de reposo
    publishFeedback(gh, ReceiveState::MOVING_ARM, "Resetting arm to resting position");
    bool reset_ok = hardware_->MoveLeftArm(
        {ARM_RESTING[0], ARM_RESTING[1], ARM_RESTING[2],
         ARM_RESTING[3], ARM_RESTING[4], ARM_RESTING[5]});

    if (reset_ok) {
        publishFeedback(gh, ReceiveState::SUCCESS, "receive_completed_for_" + item);
        return {true, "SUCCESS", item + "_received_successfully"};
    } else {
        publishFeedback(gh, ReceiveState::FAILED, "arm_failed_reset");
        return {false, "FAILED", "arm_reset_failed_after_receive"};
    }
}

// =========================================================
// publishFeedback — Formato anidado fb->feedback.message
// =========================================================
void Receive::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                              ReceiveState                       state,
                              const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[receive] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg; // TOTALMENTE ALINEADO A TU .ACTION REAL
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string Receive::stateName(ReceiveState s)
{
    switch (s) {
        case ReceiveState::READY:          return "READY";
        case ReceiveState::MOVING_ARM:     return "MOVING_ARM";
        case ReceiveState::MOVING_GRIPPER: return "MOVING_GRIPPER";
        case ReceiveState::SPEAKING:       return "SPEAKING";
        case ReceiveState::SUCCESS:        return "SUCCESS";
        case ReceiveState::FAILED:         return "FAILED";
        default:                           return "UNKNOWN";
    }
}
