#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "navigation_tools/Navigation.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class GoTo {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;

  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  GoTo(const rclcpp::Node::SharedPtr &node,
       const std::shared_ptr<Navigation> &navigation);

  bool run(const std::string &location_name,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Navigation> navigation;
  Executor executor;
};
