#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include <string>

class Unfocus
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Unfocus(const rclcpp::Node::SharedPtr& node);

    bool run(const std::string& args,
             const std::shared_ptr<GoalHandle>& goalHandle);

private:
    rclcpp::Node::SharedPtr node;
    Executor executor;
};
