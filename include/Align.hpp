#pragma once

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "executor_tools/Executor.hpp"

class Align
{
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  explicit Align(const rclcpp::Node::SharedPtr& node);

  bool run(const std::string& argument,
           const std::shared_ptr<GoalHandle>& goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
  Executor executor;
};
