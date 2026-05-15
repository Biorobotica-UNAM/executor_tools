#include "executor_tools/GoTo.hpp"

// =========================================================
// Constructor
// =========================================================
GoTo::GoTo(const rclcpp::Node::SharedPtr&     node,
           const std::shared_ptr<Navigation>& navigation)
: node_(node), navigation_(navigation)
{}

// =========================================================
// run
// =========================================================
Result GoTo::run(const std::string&                 location_name,
                 const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, GoToState::READY, location_name);

    // ── NAVIGATING ────────────────────────────────────────
    publishFeedback(gh, GoToState::NAVIGATING, location_name);

    bool ok = navigation_->GoTo(location_name);

    // ── SUCCESS / FAILED ──────────────────────────────────
    if (ok) {
        publishFeedback(gh, GoToState::SUCCESS, location_name);
        return {true, "SUCCESS", "arrived_at: " + location_name};
    } else {
        publishFeedback(gh, GoToState::FAILED, location_name);
        return {false, "FAILED", "navigation_failed: " + location_name};
    }
}

// =========================================================
// publishFeedback
// =========================================================
void GoTo::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                           GoToState                          state,
                           const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[go_to] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string GoTo::stateName(GoToState s)
{
    switch (s) {
        case GoToState::READY:      return "READY";
        case GoToState::NAVIGATING: return "NAVIGATING";
        case GoToState::SUCCESS:    return "SUCCESS";
        case GoToState::FAILED:     return "FAILED";
        default:                    return "UNKNOWN";
    }
}
