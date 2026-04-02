#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "task_interfaces/srv/take.hpp"

class Take {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;
  using TakeSrv = task_interfaces::srv::Take;

  Take(const rclcpp::Node::SharedPtr &node);

  bool run(const std::string &object_name,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<TakeSrv>::SharedPtr take_client_;
  Executor executor;
};