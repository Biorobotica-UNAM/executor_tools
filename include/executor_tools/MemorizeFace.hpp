#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MemorizeFace {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  explicit MemorizeFace(const rclcpp::Node::SharedPtr &node);

  bool run(const std::string &person_name,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  Executor executor;
};