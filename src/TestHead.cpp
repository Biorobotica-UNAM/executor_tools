#include "executor_tools/TestHead.hpp"

#include <array>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// =========================================================
// TEXTOS DE VOZ EDITABLES (Modifica aquí las frases)
// =========================================================
namespace {

constexpr const char* SPEECH_LOOK_LEFT  = "Looking to the left.";
constexpr const char* SPEECH_LOOK_RIGHT = "Looking to the right.";
constexpr const char* SPEECH_CENTER_H   = "Centering the head horizontally.";
constexpr const char* SPEECH_LOOK_UP    = "Looking up.";
constexpr const char* SPEECH_LOOK_DOWN  = "Looking down.";
constexpr const char* SPEECH_CENTER_V   = "Centering the head vertically.";

// Rangos articulares (en radianes)
constexpr double PAN_LEFT    =  0.60;
constexpr double PAN_RIGHT   = -0.60;
constexpr double TILT_UP     =  0.40;
constexpr double TILT_DOWN   = -0.70;
constexpr double PAN_CENTER  =  0.0;
constexpr double TILT_CENTER =  0.0;

constexpr auto HEAD_SETTLE = 3000ms;

struct HeadStep {
    TestHeadState state;
    double        pan;
    double        tilt;
    const char*   label;   // Log interno ROS 2
    const char*   speech;  // Frase por voz
};

constexpr std::array<HeadStep, 6> HEAD_SEQUENCE = {{
    {TestHeadState::LOOKING_LEFT,          PAN_LEFT,   TILT_CENTER, "left",   SPEECH_LOOK_LEFT},
    {TestHeadState::LOOKING_RIGHT,         PAN_RIGHT,  TILT_CENTER, "right",  SPEECH_LOOK_RIGHT},
    {TestHeadState::CENTERING_HORIZONTAL,  PAN_CENTER, TILT_CENTER, "center", SPEECH_CENTER_H},
    {TestHeadState::LOOKING_UP,            PAN_CENTER, TILT_UP,     "up",      SPEECH_LOOK_UP},
    {TestHeadState::LOOKING_DOWN,          PAN_CENTER, TILT_DOWN,   "down",    SPEECH_LOOK_DOWN},
    {TestHeadState::CENTERING_VERTICAL,    PAN_CENTER, TILT_CENTER, "center", SPEECH_CENTER_V},
}};

} // namespace

// =========================================================
// Constructor
// =========================================================
TestHead::TestHead(const rclcpp::Node::SharedPtr&    node,
                   const std::shared_ptr<Hardware>& hardware,
                   const std::shared_ptr<HRI>&      hri)
: node_(node), hardware_(hardware), hri_(hri)
{}

// =========================================================
// run
// =========================================================
Result TestHead::run(const std::string&                 argument,
                     const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, TestHeadState::READY, argument);

    for (const auto& step : HEAD_SEQUENCE) {
        if (!gh->is_active())
            return {false, "FAILED", "goal_cancelled"};

        // Feedback al supervisor
        publishFeedback(gh, step.state, step.label);

        // Locución en paralelo al movimiento
        sayAsync(step.speech);

        // Movimiento de Dynamixels
        hardware_->MoveHead({step.pan, step.tilt});

        std::this_thread::sleep_for(HEAD_SETTLE);
    }

    // ── SUCCESS ───────────────────────────────────────────
    publishFeedback(gh, TestHeadState::SUCCESS, "head_test_completed");
    return {true, "SUCCESS", "head_test_completed"};
}

// =========================================================
// sayAsync
// =========================================================
void TestHead::sayAsync(const std::string& text)
{
    if (!hri_ || text.empty()) return;

    std::thread([this, text]() {
        hri_->Say(text);
    }).detach();
}

// =========================================================
// publishFeedback
// =========================================================
void TestHead::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                               TestHeadState                      state,
                               const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[test_head] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string TestHead::stateName(TestHeadState s)
{
    switch (s) {
        case TestHeadState::READY:                 return "READY";
        case TestHeadState::LOOKING_LEFT:          return "LOOKING_LEFT";
        case TestHeadState::LOOKING_RIGHT:         return "LOOKING_RIGHT";
        case TestHeadState::CENTERING_HORIZONTAL:  return "CENTERING_HORIZONTAL";
        case TestHeadState::LOOKING_UP:            return "LOOKING_UP";
        case TestHeadState::LOOKING_DOWN:          return "LOOKING_DOWN";
        case TestHeadState::CENTERING_VERTICAL:    return "CENTERING_VERTICAL";
        case TestHeadState::SUCCESS:               return "SUCCESS";
        case TestHeadState::FAILED:                return "FAILED";
        default:                                   return "UNKNOWN";
    }
}