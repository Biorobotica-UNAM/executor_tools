#include "executor_tools/Check.hpp"

// =========================================================
// Constructor
// =========================================================
Check::Check(const rclcpp::Node::SharedPtr&   node,
             const std::shared_ptr<Hardware>& hardware)
: node_(node), hardware_(hardware)
{}

// =========================================================
// run
// =========================================================
Result Check::run(const std::string&                 argument,
                  const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, CheckState::READY, argument);

    // ── CHECKING ──────────────────────────────────────────
    publishFeedback(gh, CheckState::CHECKING, argument);

    bool ok = hardware_->CheckDoor();

    // ── SUCCESS / FAILED ──────────────────────────────────
    if (ok) {
        publishFeedback(gh, CheckState::SUCCESS, argument);
        return {true, "SUCCESS", "door_open: " + argument};
    } else {
        publishFeedback(gh, CheckState::FAILED, argument);
        return {false, "FAILED", "door_closed: " + argument};
    }
}

// =========================================================
// publishFeedback
// =========================================================
void Check::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                            CheckState                         state,
                            const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[check] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string Check::stateName(CheckState s)
{
    switch (s) {
        case CheckState::READY:    return "READY";
        case CheckState::CHECKING: return "CHECKING";
        case CheckState::SUCCESS:  return "SUCCESS";
        case CheckState::FAILED:   return "FAILED";
        default:                   return "UNKNOWN";
    }
}
