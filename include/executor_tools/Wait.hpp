#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "vision_tools/Vision.hpp"

class Wait {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Wait(const rclcpp::Node::SharedPtr &node, 
       const std::shared_ptr<Vision> &vision);

  bool run(const std::string &argument,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Vision> vision; 
  Executor executor;
};