#include "executor_tools/Give.hpp"
#include <algorithm>
#include <chrono>
#include <cctype>
#include <thread>

using namespace std::chrono_literals;

namespace {

std::string normalizeObjectToken(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::replace(value.begin(), value.end(), ' ', '_');
    return value;
}

std::string stripNumericSuffix(const std::string& value)
{
    const auto sep = value.rfind('_');
    if (sep != std::string::npos && sep + 1 < value.size() &&
        std::all_of(value.begin() + sep + 1, value.end(),
                    [](unsigned char c) { return std::isdigit(c); })) {
        return value.substr(0, sep);
    }
    return value;
}

bool objectMatches(const std::string& requested, const std::string& held)
{
    const std::string req = normalizeObjectToken(requested);
    const std::string have = normalizeObjectToken(held);
    if (req.empty() || have.empty() || have == "nothing") {
        return false;
    }
    return req == have || req == stripNumericSuffix(have) ||
           stripNumericSuffix(req) == have;
}

} // namespace

// =========================================================
// Constructor
// =========================================================
Give::Give(const rclcpp::Node::SharedPtr& node,
           const std::shared_ptr<Hardware>& hardware,
           const std::shared_ptr<Knowledge>& knowledge,
           const std::shared_ptr<HRI>& hri)
: node_(node),
  hardware_(hardware),
  knowledge_(knowledge),
  hri_(hri)
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

    // 1. Revisar en qué mano tiene el objeto
    publishFeedback(gh, GiveState::SELECTING_HAND);

    std::string hand = selectHand(object_name);

    if (hand != "right_hand" && hand != "left_hand") {
        publishFeedback(gh, GiveState::FAILED, "not_object");
        return {false, "FAILED", "not_object"};
    } 

    // 2. Extender el brazo inmediatamente 
    publishFeedback(gh, GiveState::MOVING_ARM, "Extending arm to give " + item);
    hardware_->MoveBase(0.0, 0.0, 0.0);
    
    // TODO: Verify pose works for both arms
    bool arm_ok = (hand == "left_hand") ? hardware_->MoveLeftArm(
                                            {LEFT_ARM_POINTING[0], LEFT_ARM_POINTING[1], LEFT_ARM_POINTING[2],
                                             LEFT_ARM_POINTING[3], LEFT_ARM_POINTING[4], LEFT_ARM_POINTING[5]})
                                        : hardware_->MoveRightArm(
                                            {RIGHT_ARM_POINTING[0], RIGHT_ARM_POINTING[1], RIGHT_ARM_POINTING[2],
                                             RIGHT_ARM_POINTING[3], RIGHT_ARM_POINTING[4], RIGHT_ARM_POINTING[5]});

    if (!arm_ok) {
        publishFeedback(gh, GiveState::FAILED, "arm_failed_extension");
        return {false, "FAILED", "arm_extension_failed"};
    }

    // 3. Decir frase usando el nombre del objeto recibido
    publishFeedback(gh, GiveState::SPEAKING, "Requesting retrieval of: " + item);
    std::string phrase = "Please take the " + item + ". I will open my hand in 5. 4. 3. 2. 1.";
    hri_->Say(phrase);

    // 4. Abrir la mano 
    publishFeedback(gh, GiveState::MOVING_GRIPPER, "Opening gripper");
    bool gripper_ok = (hand == "left_hand") ? hardware_->MoveLeftGripper("open")
                                            : hardware_->MoveRightGripper("open");

    if (!gripper_ok) {
        publishFeedback(gh, GiveState::FAILED, "gripper_open_failed");
        return {false, "FAILED", "gripper_open_failed"};
    }

    // 5. Actualizar la disponibilidad de la mano en el estado del mundo 
    publishFeedback(gh, GiveState::REPORTING_OCCUPANCY, "Opening gripper");
    bool report_ok = knowledge_->update_world_state(
                        "robot", "",
                        hand, "nothing");

    if (!report_ok) {
        publishFeedback(gh, GiveState::FAILED, "report_object_failed");
        return {false, "FAILED", "report_object_failed"};
    }

    // 6. Esperar 2 segundos mientras la persona retira el objeto
    publishFeedback(gh, GiveState::WAITING, "Waiting 2 seconds");
    std::this_thread::sleep_for(2s);

    // 7. Cerrar la mano
    publishFeedback(gh, GiveState::MOVING_GRIPPER, "Closing gripper");
    gripper_ok = (hand == "left_hand") ? hardware_->MoveLeftGripper("close")
                                            : hardware_->MoveRightGripper("close");

    if (!gripper_ok) {
        publishFeedback(gh, GiveState::FAILED, "gripper_close_failed");
        return {false, "FAILED", "gripper_close_failed"};
    }

    // 8. Regresar el brazo a la posición de reposo
    publishFeedback(gh, GiveState::MOVING_ARM, "Resetting arm to resting position");
    std::this_thread::sleep_for(500ms);

    bool reset_ok = (hand == "left_hand") ? hardware_->MoveLeftArm(
                                            {ARM_RESTING[0], ARM_RESTING[1], ARM_RESTING[2],
                                             ARM_RESTING[3], ARM_RESTING[4], ARM_RESTING[5]})
                                          : hardware_->MoveRightArm(
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

std::string Give::selectHand(const std::string& obj)
{
    std::string result;

    json left_object = knowledge_->get_world_state(
                                    "robot", "", "left_hand");
    json right_object = knowledge_->get_world_state(
                                    "robot", "", "right_hand");

    result.clear();

    const std::string left =
        left_object.is_string() ? left_object.get<std::string>() : "";
    const std::string right =
        right_object.is_string() ? right_object.get<std::string>() : "";

    if (objectMatches(obj, left)) result = "left_hand";
    if (objectMatches(obj, right)) result = "right_hand";

    return result;
}

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

std::string Give::stateName(GiveState s)
{
    switch (s) {
        case GiveState::READY:                  return "READY";
        case GiveState::SELECTING_HAND:         return "SELECTING_HAND";
        case GiveState::MOVING_ARM:             return "MOVING_ARM";
        case GiveState::SPEAKING:               return "SPEAKING";
        case GiveState::MOVING_GRIPPER:         return "MOVING_GRIPPER";
        case GiveState::REPORTING_OCCUPANCY:    return "REPORTING_OCCUPANCY";
        case GiveState::WAITING:                return "WAITING";
        case GiveState::SUCCESS:                return "SUCCESS";
        case GiveState::FAILED:                 return "FAILED";
        default:                                return "UNKNOWN";
    }
}
