#include "executor_tools/Saveloc.hpp"

Saveloc::Saveloc(const rclcpp::Node::SharedPtr&     node,
                 const std::shared_ptr<Navigation>& navigation)
    : node_(node), navigation_(navigation) {}

Result Saveloc::run(const std::string&                 location_name,
                     const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    publishFeedback(gh, SavelocState::READY, location_name);
    publishFeedback(gh, SavelocState::SAVING, location_name);

    // Delegar la lógica de guardado a Navigation (igual que GoTo usa GoToDetailed)
    bool success = navigation_->AddLocation(location_name);

    if (success) {
        publishFeedback(gh, SavelocState::SUCCESS, location_name);
        return {true, "SUCCESS", "location_saved: " + location_name};
    } else {
        publishFeedback(gh, SavelocState::FAILED, location_name + ": save_failed");
        return {false, "FAILED", "save_failed: " + location_name};
    }
}

void Saveloc::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                              SavelocState                       state,
                              const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[saveloc] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

std::string Saveloc::stateName(SavelocState s)
{
    switch (s) {
        case SavelocState::READY:   return "READY";
        case SavelocState::SAVING:  return "SAVING";
        case SavelocState::SUCCESS: return "SUCCESS";
        case SavelocState::FAILED:  return "FAILED";
        default:                    return "UNKNOWN";
    }
}