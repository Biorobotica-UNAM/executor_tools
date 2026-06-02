#include "executor_tools/GoToMapless.hpp"
#include <chrono>

using namespace std::chrono_literals;

GoToMapless::GoToMapless(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    mapless_client_ = node_->create_client<NavigateMapless>("/navigation/mapless_goto");

    goal_reached_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/maples_nav/goal_reached", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data && !goal_received_) {
                goal_received_ = true;
                goal_reached_promise_.set_value(true);
            }
        });
}

Result GoToMapless::run(const std::string&                 target_frame,
                         const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    publishFeedback(gh, GoToMaplessState::READY, target_frame);

    // --- 1. Activar destino llamando al servicio ---
    publishFeedback(gh, GoToMaplessState::CALLING_SERVICE, target_frame);

    if (!mapless_client_->wait_for_service(3s)) {
        publishFeedback(gh, GoToMaplessState::FAILED, "service_not_available");
        return {false, "FAILED", "mapless_goto_service_unavailable"};
    }

    auto request = std::make_shared<NavigateMapless::Request>();
    request->target_frame = target_frame;

    auto future = mapless_client_->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
        publishFeedback(gh, GoToMaplessState::FAILED, "service_call_timeout");
        return {false, "FAILED", "mapless_goto_service_timeout"};
    }

    auto response = future.get();
    if (!response->success) {
        publishFeedback(gh, GoToMaplessState::FAILED, response->message);
        return {false, "FAILED", "mapless_goto_rejected: " + response->message};
    }

    // --- 2. Esperar goal_reached ---
    publishFeedback(gh, GoToMaplessState::NAVIGATING, target_frame);

    // Reiniciar la promesa para esta ejecución (el hilo es nuevo)
    goal_received_ = false;
    goal_reached_promise_ = std::promise<bool>();
    goal_reached_future_ = goal_reached_promise_.get_future();

    constexpr auto timeout = 60s;
    if (goal_reached_future_.wait_for(timeout) == std::future_status::ready) {
        publishFeedback(gh, GoToMaplessState::SUCCESS, target_frame);
        return {true, "SUCCESS", "arrived_mapless: " + target_frame};
    } else {
        publishFeedback(gh, GoToMaplessState::TIMEOUT, target_frame);
        return {false, "TIMEOUT", "mapless_navigation_timeout: " + target_frame};
    }
}

void GoToMapless::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                                  GoToMaplessState                   state,
                                  const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[goto_mapless] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

std::string GoToMapless::stateName(GoToMaplessState s)
{
    switch (s) {
        case GoToMaplessState::READY:           return "READY";
        case GoToMaplessState::CALLING_SERVICE: return "CALLING_SERVICE";
        case GoToMaplessState::NAVIGATING:      return "NAVIGATING";
        case GoToMaplessState::SUCCESS:         return "SUCCESS";
        case GoToMaplessState::FAILED:          return "FAILED";
        case GoToMaplessState::TIMEOUT:         return "TIMEOUT";
        default:                                return "UNKNOWN";
    }
}