#pragma once

#include "rclcpp/rclcpp.hpp"
#include "supervisor_executor_interfaces/action/execute_task.hpp"
#include "executor_tools/ExecutorClient.hpp"
#include "align_interfaces/srv/align.hpp" 

class Align {
public:
  using ExecuteTask = supervisor_executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;
  using AlignSrv = align_interfaces::srv::Align; 

  Align(const rclcpp::Node::SharedPtr &node);

  bool run(const std::string &target_name,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  ExecutorClient executor;
  rclcpp::Client<AlignSrv>::SharedPtr align_client_;
};