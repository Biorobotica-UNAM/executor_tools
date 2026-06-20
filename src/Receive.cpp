#include "executor_tools/Receive.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Receive::Receive(const rclcpp::Node::SharedPtr& node,
                 const std::shared_ptr<Hardware>& hardware,
                 const std::shared_ptr<HRI>& hri,
                 const std::shared_ptr<Knowledge>& knowledge)
: node_(node), 
  hardware_(hardware),
  hri_(hri),
  knowledge_(knowledge)
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

    // 1. Verifica que el brazo izquierdo esté disponible 
    publishFeedback(gh, ReceiveState::VERIFYING_VACANCY);

    json left_object = knowledge_->get_world_state(
                                    "robot", "", "left_hand");

    json right_object = knowledge_->get_world_state(
                                    "robot", "", "right_hand"); 
    
    std::string hand = "left_hand";

    if (left_object.get<std::string>() != "nothing" &&
        right_object.get<std::string>() != "nothing") {
        publishFeedback(gh, ReceiveState::FAILED, "arm_not_available");
        return {false, "FALSE", "arm_not_available"};
    }

    if (left_object.get<std::string>() != "nothing")
        hand = "right_hand";

    // 2. Extender el brazo inmediatamente 
    publishFeedback(gh, ReceiveState::MOVING_ARM, "Extending " + hand + " to receive " + item);
    bool arm_ok = (hand == "left_hand") ? hardware_->MoveLeftArm(
        {LEFT_ARM_POINTING[0], LEFT_ARM_POINTING[1], LEFT_ARM_POINTING[2],
         LEFT_ARM_POINTING[3], LEFT_ARM_POINTING[4], LEFT_ARM_POINTING[5]})
        : hardware_->MoveRightArm(
        {RIGHT_ARM_POINTING[0], RIGHT_ARM_POINTING[1], RIGHT_ARM_POINTING[2],
         RIGHT_ARM_POINTING[3], RIGHT_ARM_POINTING[4], RIGHT_ARM_POINTING[5]});

    if (!arm_ok) {
        publishFeedback(gh, ReceiveState::FAILED, "arm_failed_extension");
        return {false, "FAILED", "arm_extension_failed"};
    }

    // 3. Abrir el Gripper 
    publishFeedback(gh, ReceiveState::OPENING_GRIPPER, "Opening gripper to receive " + item);
    bool gripper_ok = (hand == "left_hand") ? hardware_->MoveLeftGripper("open")
                                            : hardware_->MoveRightGripper("open");

    if (!gripper_ok) {
        publishFeedback(gh, ReceiveState::FAILED, "gripper_open_failed");
        return {false, "FAILED", "gripper_open_failed"};
    }

    std::this_thread::sleep_for(200ms); 

    // 4. Hablar 
    publishFeedback(gh, ReceiveState::SPEAKING, "Requesting placement of: " + item);
    std::string phrase = "Please place the " + item + " in my hand. I will close my hand in 5. 4. 3. 2. 1.";
    hri_->Say(phrase);

    // 5. Cerrar la mano 
    publishFeedback(gh, ReceiveState::CLOSING_GRIPPER, "Closing gripper on " + item);
    gripper_ok = (hand == "left_hand") ? hardware_->MoveLeftGripper("close")
                                        : hardware_->MoveRightGripper("close");

    if (!gripper_ok) {
        publishFeedback(gh, ReceiveState::FAILED, "gripper_close_failed");
        return {false, "FAILED", "gripper_close_failed"};
    }
    std::this_thread::sleep_for(1s); 

    // 6. Registrar el objeto que tiene en la mano
    publishFeedback(gh, ReceiveState::REPORTING_OCCUPANCY);

    bool report_ok = knowledge_->update_world_state(
                        "robot", "",
                        hand, object_name);

    if (!report_ok) {
        publishFeedback(gh, ReceiveState::FAILED, "report_object_failed");
        return {false, "FAILED", "report_object_failed"};
    }

    // 7. Regresar el brazo a la posición de reposo
    publishFeedback(gh, ReceiveState::MOVING_ARM, "Sending arm to transport position");
    bool reset_ok = hardware_->MoveLeftArm(
        {ARM_TRANSPORT[0], ARM_TRANSPORT[1], ARM_TRANSPORT[2],
         ARM_TRANSPORT[3], ARM_TRANSPORT[4], ARM_TRANSPORT[5]});

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
        case ReceiveState::READY:               return "READY";
        case ReceiveState::VERIFYING_VACANCY:   return "VERIFYING_VACANCY";
        case ReceiveState::MOVING_ARM:          return "MOVING_ARM";
        case ReceiveState::OPENING_GRIPPER:     return "OPENING_GRIPPER";
        case ReceiveState::SPEAKING:            return "SPEAKING";
        case ReceiveState::CLOSING_GRIPPER:     return "CLOSING_GRIPPER";
        case ReceiveState::REPORTING_OCCUPANCY: return "REPORTING_OCCUPANCY";
        case ReceiveState::SUCCESS:             return "SUCCESS";
        case ReceiveState::FAILED:              return "FAILED";
        default:                                return "UNKNOWN";
    }
}
