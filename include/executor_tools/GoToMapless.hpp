#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "known_locations_interfaces/srv/navigate_mapless.hpp"
#include "std_msgs/msg/bool.hpp"

#include <string>
#include <future>

enum class GoToMaplessState {
    READY,
    CALLING_SERVICE,
    NAVIGATING,
    SUCCESS,
    FAILED,
    TIMEOUT
};

class GoToMapless
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;
    using NavigateMapless = known_locations_interfaces::srv::NavigateMapless;

    explicit GoToMapless(const rclcpp::Node::SharedPtr& node);

    Result run(const std::string&                 target_frame,
               const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         GoToMaplessState                   state,
                         const std::string&                 detail = "");

    static std::string stateName(GoToMaplessState s);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<NavigateMapless>::SharedPtr mapless_client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;

    std::promise<bool> goal_reached_promise_;
    std::future<bool> goal_reached_future_;
    bool goal_received_{false};
};