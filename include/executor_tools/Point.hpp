#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "hardware_tools/Hardware.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Point {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Point(const rclcpp::Node::SharedPtr &node,
        const std::shared_ptr<Hardware> &hardware);

  bool run(const std::string &target_name,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Hardware> hardware;
  Executor executor;
};