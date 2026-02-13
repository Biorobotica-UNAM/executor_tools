#pragma once

#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"

#include "executor_tools/RequestAuthorization.hpp"

class Listen
{
public:
  using ExecuteTask =
      executor_interfaces::action::ExecuteTask;

  using GoalHandle =
      rclcpp_action::ServerGoalHandle<ExecuteTask>;

  explicit Listen(const rclcpp::Node::SharedPtr& node);

  bool run(
    const std::string& grammar,
    const std::shared_ptr<GoalHandle>& goal_handle);

private:
  rclcpp::Node::SharedPtr node_;
  RequestAuthorization auth_;
};
