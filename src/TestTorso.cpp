#include "executor_tools/TestTorso.hpp"

#include <array>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace {

constexpr double TORSO_BOTTOM = 0.10;
constexpr double TORSO_MIDDLE = 0.20;
constexpr double TORSO_TOP    = 0.30;

constexpr auto TORSO_SETTLE = 4000ms;

struct TorsoStep {
    TestTorsoState state;
    double         position;
    const char*    label;
    const char*    speech;
};

constexpr std::array<TorsoStep, 3> TORSO_SEQUENCE = {{
    {TestTorsoState::MOVING_MIDDLE, TORSO_MIDDLE, "middle", "I am going to start the torso test."},
    {TestTorsoState::MOVING_BOTTOM, TORSO_BOTTOM, "bottom", "I am lowering my torso."},
    {TestTorsoState::MOVING_TOP,    TORSO_TOP,    "top",    "I am raising my trunk."},
}};

}

// =========================================================
// Constructor
// =========================================================
TestTorso::TestTorso(const rclcpp::Node::SharedPtr&    node,
                     const std::shared_ptr<Hardware>& hardware,
                     const std::shared_ptr<HRI>&      hri)
: node_(node), hardware_(hardware), hri_(hri)
{}

// =========================================================
// run
// =========================================================
Result TestTorso::run(const std::string&                 argument,
                      const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, TestTorsoState::READY, argument);

    for (const auto& step : TORSO_SEQUENCE) {
        if (!gh->is_active())
            return {false, "FAILED", "goal_cancelled"};

        publishFeedback(gh, step.state, step.label);

        // Anuncio informativo por voz en paralelo al movimiento
        sayAsync(step.speech);

        if (!hardware_->MoveTorso(step.position)) {
            const std::string reason = std::string("torso_failed_") + step.label;
            publishFeedback(gh, TestTorsoState::FAILED, reason);
            return {false, "FAILED", reason};
        }

        std::this_thread::sleep_for(TORSO_SETTLE);
    }

    // ── SUCCESS ───────────────────────────────────────────
    publishFeedback(gh, TestTorsoState::SUCCESS, "torso_test_completed");
    return {true, "SUCCESS", "torso_test_completed"};
}

// =========================================================
// sayAsync
// =========================================================
void TestTorso::sayAsync(const std::string& text)
{
    if (!hri_ || text.empty()) return;

    std::thread([this, text]() {
        hri_->Say(text);
    }).detach();
}

// =========================================================
// publishFeedback
// =========================================================
void TestTorso::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                                TestTorsoState                     state,
                                const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[test_torso] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string TestTorso::stateName(TestTorsoState s)
{
    switch (s) {
        case TestTorsoState::READY:         return "READY";
        case TestTorsoState::MOVING_MIDDLE: return "MOVING_MIDDLE";
        case TestTorsoState::MOVING_BOTTOM: return "MOVING_BOTTOM";
        case TestTorsoState::MOVING_TOP:    return "MOVING_TOP";
        case TestTorsoState::SUCCESS:       return "SUCCESS";
        case TestTorsoState::FAILED:        return "FAILED";
        default:                            return "UNKNOWN";
    }
}