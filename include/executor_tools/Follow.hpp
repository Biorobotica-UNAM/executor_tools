#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"

#include <string>

// =========================================================
// Estados de la FSM para Follow
// =========================================================
enum class FollowState {
    READY,
    PENDING_SERVICE,
    SUCCESS,
    FAILED
};

class Follow
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Follow(const rclcpp::Node::SharedPtr& node);

    Result run(const std::string& argument,
               const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         FollowState state,
                         const std::string& detail = "");

    static std::string stateName(FollowState s);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};