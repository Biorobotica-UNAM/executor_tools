#include "executor_tools/Point.hpp"

#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Point::Point(const rclcpp::Node::SharedPtr&   node,
             const std::shared_ptr<Hardware>& hardware)
: node_(node), hardware_(hardware)
{}

// =========================================================
// run — despacha según argumento
// =========================================================
Result Point::run(const std::string&                 argument,
                  const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, PointState::READY, argument);

    if (argument == "enable")  return enable(gh);
    if (argument == "disable") return disable(gh);

    return {false, "FAILED", "unknown_argument: " + argument};
}

// =========================================================
// enable — alinea base + centra cabeza + extiende brazo
// =========================================================
Result Point::enable(const std::shared_ptr<GoalHandle>& gh)
{
    // ── ALIGNING ──────────────────────────────────────────
    publishFeedback(gh, PointState::ALIGNING);

    // 1. Leer pan actual de la cabeza
    const double pan   = hardware_->GetCurrentPose("head", 0);
    const double tilt  = hardware_->GetCurrentPose("head", 1);

    // 2. Calcular duración del giro para alinear base con pan
    if (std::abs(pan) > 1e-3)
    {
        const double duration_s = std::abs(pan / ANGULAR_SPEED) + ALIGN_EXTRA;
        const double sign       = (pan < 0.0) ? -1.0 : 1.0;

        rclcpp::Rate rate(30.0);
        const auto   start    = node_->now();
        double       cur_pan  = pan;
        const double alpha    = 0.1;

        while (rclcpp::ok() &&
               (node_->now() - start).seconds() < duration_s)
        {
            if (!gh->is_active())
                return {false, "FAILED", "goal_cancelled_during_align"};

            // Suavizar cabeza hacia 0 mientras gira la base
            cur_pan = cur_pan + alpha * (0.0 - cur_pan);
            hardware_->MoveHead({cur_pan, tilt});
            hardware_->MoveBase(0.0, 0.0, sign * ANGULAR_SPEED);

            rate.sleep();
        }
    }

    // 3. Parar base y centrar cabeza
    hardware_->MoveBase(0.0, 0.0, 0.0);
    hardware_->MoveHead({0.0, tilt});

    // ── POINTING ──────────────────────────────────────────
    publishFeedback(gh, PointState::POINTING);

    bool ok = hardware_->MoveLeftArm(
        {ARM_POINTING[0], ARM_POINTING[1], ARM_POINTING[2],
         ARM_POINTING[3], ARM_POINTING[4], ARM_POINTING[5]});

    if (ok) {
        publishFeedback(gh, PointState::SUCCESS, "enable");
        return {true, "SUCCESS", "pointing_enabled"};
    } else {
        publishFeedback(gh, PointState::FAILED, "arm_failed");
        return {false, "FAILED", "arm_failed_during_pointing"};
    }
}

// =========================================================
// disable — baja brazo
// =========================================================
Result Point::disable(const std::shared_ptr<GoalHandle>& gh)
{
    // ── RESETTING ─────────────────────────────────────────
    publishFeedback(gh, PointState::RESETTING);

    hardware_->MoveBase(0.0, 0.0, 0.0);

    bool ok = hardware_->MoveLeftArm(
        {ARM_RESTING[0], ARM_RESTING[1], ARM_RESTING[2],
         ARM_RESTING[3], ARM_RESTING[4], ARM_RESTING[5]});

    if (ok) {
        publishFeedback(gh, PointState::SUCCESS, "disable");
        return {true, "SUCCESS", "pointing_disabled"};
    } else {
        publishFeedback(gh, PointState::FAILED, "arm_failed");
        return {false, "FAILED", "arm_failed_during_reset"};
    }
}

// =========================================================
// publishFeedback
// =========================================================
void Point::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                            PointState                         state,
                            const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[point] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string Point::stateName(PointState s)
{
    switch (s) {
        case PointState::READY:    return "READY";
        case PointState::ALIGNING: return "ALIGNING";
        case PointState::POINTING: return "POINTING";
        case PointState::RESETTING:return "RESETTING";
        case PointState::SUCCESS:  return "SUCCESS";
        case PointState::FAILED:   return "FAILED";
        default:                   return "UNKNOWN";
    }
}
