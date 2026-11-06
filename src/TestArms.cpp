#include "executor_tools/TestArms.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace {

constexpr auto ARM_SETTLE  = 1500ms;
constexpr auto WAVE_PAUSE  = 400ms;

}

// =========================================================
// Constructor
// =========================================================
TestArms::TestArms(const rclcpp::Node::SharedPtr&   node,
                   const std::shared_ptr<Hardware>& hardware,
                   const std::shared_ptr<HRI>&      hri)
: node_(node), hardware_(hardware), hri_(hri)
{}

// =========================================================
// run
// =========================================================
Result TestArms::run(const std::string&                 argument,
                     const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, TestArmsState::READY, argument);

    // Primero el brazo izquierdo, luego el derecho
    for (const bool is_left : {true, false}) {
        const std::string side = is_left ? "left" : "right";

        const TestArmsState raising  = is_left ? TestArmsState::RAISING_LEFT_ARM
                                               : TestArmsState::RAISING_RIGHT_ARM;
        const TestArmsState waving   = is_left ? TestArmsState::WAVING_LEFT_ARM
                                               : TestArmsState::WAVING_RIGHT_ARM;
        const TestArmsState lowering = is_left ? TestArmsState::LOWERING_LEFT_ARM
                                               : TestArmsState::LOWERING_RIGHT_ARM;

        const auto& raised  = is_left ? LEFT_ARM_RAISED  : RIGHT_ARM_RAISED;
        const auto& resting = is_left ? LEFT_ARM_RESTING : RIGHT_ARM_RESTING;

        // 1. Subir el brazo
        if (!gh->is_active())
            return {false, "FAILED", "goal_cancelled"};

        publishFeedback(gh, raising, side);
        if (!moveArm(is_left, raised)) {
            publishFeedback(gh, TestArmsState::FAILED, side + "_arm_raise_failed");
            return {false, "FAILED", side + "_arm_raise_failed"};
        }
        std::this_thread::sleep_for(ARM_SETTLE);

        // 2. Saludar (movimiento + voz )
        publishFeedback(gh, waving, side);

        if (hri_) {
            std::string greeting = is_left ? "I am moving my left arm." : "I am moving my right arm.";
            std::thread([this, greeting]() {
                hri_->Say(greeting);
            }).detach();
        }

        if (!waveArm(gh, is_left)) {
            publishFeedback(gh, TestArmsState::FAILED, side + "_arm_wave_failed");
            return {false, "FAILED", side + "_arm_wave_failed"};
        }

        // 3. Regresar a reposo
        publishFeedback(gh, lowering, side);
        if (!moveArm(is_left, resting)) {
            publishFeedback(gh, TestArmsState::FAILED, side + "_arm_lower_failed");
            return {false, "FAILED", side + "_arm_lower_failed"};
        }
        std::this_thread::sleep_for(ARM_SETTLE);
    }

    // ── 4. Regresar ambos brazos a pose inicial ───────────
    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    if (!moveArm(true, LEFT_ARM_INITIAL)) {
        publishFeedback(gh, TestArmsState::FAILED, "left_arm_initial_failed");
        return {false, "FAILED", "left_arm_initial_failed"};
    }

    if (!moveArm(false, RIGHT_ARM_INITIAL)) {
        publishFeedback(gh, TestArmsState::FAILED, "right_arm_initial_failed");
        return {false, "FAILED", "right_arm_initial_failed"};
    }
    std::this_thread::sleep_for(ARM_SETTLE);

    // ── SUCCESS ───────────────────────────────────────────
    publishFeedback(gh, TestArmsState::SUCCESS, "arms_test_completed");
    return {true, "SUCCESS", "arms_test_completed"};
}

// =========================================================
// moveArm
// =========================================================
bool TestArms::moveArm(bool is_left, const std::array<double, 6>& pose)
{
    return is_left ? hardware_->MoveLeftArm(
                         {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]})
                   : hardware_->MoveRightArm(
                         {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]});
}

bool TestArms::waveArm(const std::shared_ptr<GoalHandle>& gh, bool is_left)
{
    const auto& base = is_left ? LEFT_ARM_RAISED : RIGHT_ARM_RAISED;

    for (std::size_t k = 0; k < WAVE_JOINTS.size(); ++k) {
        const int    joint = WAVE_JOINTS[k];
        const double amp   = WAVE_AMPLITUDES[k];

        for (int cycle = 0; cycle < WAVE_CYCLES; ++cycle) {
            for (const double sign : {1.0, -1.0}) {
                if (!gh->is_active())
                    return false;

                auto pose = base;
                pose[joint] = base[joint] + sign * amp;

                if (!moveArm(is_left, pose))
                    return false;

                std::this_thread::sleep_for(WAVE_PAUSE);
            }
        }

        if (!gh->is_active())
            return false;

        if (!moveArm(is_left, base))
            return false;

        std::this_thread::sleep_for(WAVE_PAUSE);
    }

    return true;
}

// =========================================================
// publishFeedback
// =========================================================
void TestArms::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                               TestArmsState                      state,
                               const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[test_arms] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string TestArms::stateName(TestArmsState s)
{
    switch (s) {
        case TestArmsState::READY:              return "READY";
        case TestArmsState::RAISING_LEFT_ARM:   return "RAISING_LEFT_ARM";
        case TestArmsState::WAVING_LEFT_ARM:    return "WAVING_LEFT_ARM";
        case TestArmsState::LOWERING_LEFT_ARM:  return "LOWERING_LEFT_ARM";
        case TestArmsState::RAISING_RIGHT_ARM:  return "RAISING_RIGHT_ARM";
        case TestArmsState::WAVING_RIGHT_ARM:   return "WAVING_RIGHT_ARM";
        case TestArmsState::LOWERING_RIGHT_ARM: return "LOWERING_RIGHT_ARM";
        case TestArmsState::SUCCESS:            return "SUCCESS";
        case TestArmsState::FAILED:             return "FAILED";
        default:                                return "UNKNOWN";
    }
}
