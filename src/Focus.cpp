#include "executor_tools/Focus.hpp"

// =========================================================
// Constructor
// =========================================================
Focus::Focus(const rclcpp::Node::SharedPtr& node,
            const std::shared_ptr<Vision>& vision)
: node_(node), vision_(vision)
{}

// =========================================================
// run
// =========================================================
Result Focus::run(const std::string& argument,
                  const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    publishFeedback(gh, FocusState::READY, argument);

    bool focus_value = parseArgument(argument);

    publishFeedback(gh, FocusState::FOCUSING, argument);

    bool ok = vision_->Focus(focus_value);

    if (ok) {
        publishFeedback(gh, FocusState::SUCCESS, argument);
        return {true, "SUCCESS", "focus_updated"};
    } else {
        publishFeedback(gh, FocusState::FAILED, argument);
        return {false, "FAILED", "focus_failed"};
    }
}

// =========================================================
// parseArgument
// =========================================================
bool Focus::parseArgument(const std::string& argument)
{
    if (argument == "true"  ||
        argument == "on"    ||
        argument == "1"     ||
        argument == "start" ||
        argument == "enable")
    {
        return true;
    }

    return false;
}

// =========================================================
// publishFeedback
// =========================================================
void Focus::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                            FocusState state,
                            const std::string& detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[focus] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string Focus::stateName(FocusState s)
{
    switch (s) {
        case FocusState::READY:            return "READY";
        case FocusState::FOCUSING:         return "FOCUSING";
        case FocusState::SUCCESS:          return "SUCCESS";
        case FocusState::FAILED:           return "FAILED";
        default:                           return "UNKNOWN";
    }
}
