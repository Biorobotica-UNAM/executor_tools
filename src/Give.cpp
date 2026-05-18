#include "executor_tools/Give.hpp"
#include <chrono>
#include <cmath>
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
// run — despacha según argumento
// =========================================================
Result Give::run(const std::string& argument,
                 const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, GiveState::READY, argument);

    if (argument == "enable")  return enable(gh);
    if (argument == "disable") return disable(gh);

    return {false, "FAILED", "unknown_argument: " + argument};
}

// =========================================================
// enable — Recibir la bolsa
// =========================================================
Result Give::enable(const std::shared_ptr<GoalHandle>& gh)
{
    // 1. Coreografía de alineación (Cuerpo + Cabeza)
    publishFeedback(gh, GiveState::ALIGNING);
    const double pan   = hardware_->GetCurrentPose("head", 0);
    const double tilt  = hardware_->GetCurrentPose("head", 1);

    if (std::abs(pan) > 1e-3)
    {
        const double duration_s = std::abs(pan / ANGULAR_SPEED) + ALIGN_EXTRA;
        const double sign       = (pan < 0.0) ? -1.0 : 1.0;

        rclcpp::Rate rate(30.0);
        const auto start = node_->now();
        double cur_pan = pan;
        const double alpha = 0.1;

        while (rclcpp::ok() && (node_->now() - start).seconds() < duration_s)
        {
            if (!gh->is_active())
                return {false, "FAILED", "goal_cancelled_during_align"};

            cur_pan = cur_pan + alpha * (0.0 - cur_pan);
            hardware_->MoveHead({cur_pan, tilt});
            hardware_->MoveBase(0.0, 0.0, sign * ANGULAR_SPEED);
            rate.sleep();
        }
    }
    hardware_->MoveBase(0.0, 0.0, 0.0);
    hardware_->MoveHead({0.0, tilt});

    // 2. Extender el brazo
    publishFeedback(gh, GiveState::MOVING_ARM, "Extending arm");
    bool arm_ok = hardware_->MoveLeftArm(
        {ARM_POINTING[0], ARM_POINTING[1], ARM_POINTING[2],
         ARM_POINTING[3], ARM_POINTING[4], ARM_POINTING[5]});

    if (!arm_ok) {
        publishFeedback(gh, GiveState::FAILED, "arm_failed_extension");
        return {false, "FAILED", "arm_extension_failed"};
    }

    // 3. Abrir el Gripper
    //hardware_->OpenGripper();
    std::this_thread::sleep_for(500ms); // Breve pausa para asegurar apertura física

    // 4. Hablar
    publishFeedback(gh, GiveState::SPEAKING, "Requesting bag placement");
    hri_->Say("Please place the bag in my hand. I will close my hand in 5, 4, 3, 2, 1.");

    // 5. Cerrar la mano
    publishFeedback(gh, GiveState::MOVING_ARM, "Closing gripper");
    //hardware_->CloseGripper();
    std::this_thread::sleep_for(1s); // Tiempo para asegurar el agarre firme

    publishFeedback(gh, GiveState::SUCCESS, "enable_completed");
    return {true, "SUCCESS", "bag_received_successfully"};
}

// =========================================================
// disable — Entregar la bolsa y guardar brazo
// =========================================================
Result Give::disable(const std::shared_ptr<GoalHandle>& gh)
{
    // 1. Extender el brazo hacia el frente (en caso de que estuviera guardado)
    publishFeedback(gh, GiveState::MOVING_ARM, "Extending arm to give");
    hardware_->MoveBase(0.0, 0.0, 0.0);
    bool arm_ok = hardware_->MoveLeftArm(
        {ARM_POINTING[0], ARM_POINTING[1], ARM_POINTING[2],
         ARM_POINTING[3], ARM_POINTING[4], ARM_POINTING[5]});

    if (!arm_ok) {
        publishFeedback(gh, GiveState::FAILED, "arm_failed_extension");
        return {false, "FAILED", "arm_extension_failed"};
    }

    // 2. Decir frase en inglés
    publishFeedback(gh, GiveState::SPEAKING, "Requesting bag retrieval");
    hri_->Say("Please take the bag. I will open my hand in 5, 4, 3, 2, 1.");

    // 3. Abrir la mano
    publishFeedback(gh, GiveState::MOVING_ARM, "Opening gripper");
    //hardware_->OpenGripper();

    // 4. Esperar 2 segundos mientras la persona retira la bolsa
    publishFeedback(gh, GiveState::WAITING, "Waiting 2 seconds");
    std::this_thread::sleep_for(2s);

    // 5. Cerrar mano y regresar a posición de reposo
    publishFeedback(gh, GiveState::MOVING_ARM, "Resetting arm to resting position");
    //hardware_->CloseGripper(); // Cierra pinza para navegar compacto
    std::this_thread::sleep_for(500ms);

    bool reset_ok = hardware_->MoveLeftArm(
        {ARM_RESTING[0], ARM_RESTING[1], ARM_RESTING[2],
         ARM_RESTING[3], ARM_RESTING[4], ARM_RESTING[5]});

    if (reset_ok) {
        publishFeedback(gh, GiveState::SUCCESS, "disable_completed");
        return {true, "SUCCESS", "bag_delivered_successfully"};
    } else {
        publishFeedback(gh, GiveState::FAILED, "arm_failed_reset");
        return {false, "FAILED", "arm_reset_failed"};
    }
}

// =========================================================
// publishFeedback — Formato anidado alineado a tu .action
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
std::string Give::stateName(GiveState s)
{
    switch (s) {
        case GiveState::READY:      return "READY";
        case GiveState::ALIGNING:   return "ALIGNING";
        case GiveState::MOVING_ARM: return "MOVING_ARM";
        case GiveState::SPEAKING:   return "SPEAKING";
        case GiveState::WAITING:    return "WAITING";
        case GiveState::SUCCESS:    return "SUCCESS";
        case GiveState::FAILED:     return "FAILED";
        default:                    return "UNKNOWN";
    }
}