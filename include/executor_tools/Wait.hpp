#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"

class Wait {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Wait(const rclcpp::Node::SharedPtr &node);

  bool run(const std::string &argument,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  Executor executor;
  
  // Variable interna para la detección
  bool person_detected_;
  void detection_callback(const std_msgs::msg::Bool::SharedPtr msg);
};