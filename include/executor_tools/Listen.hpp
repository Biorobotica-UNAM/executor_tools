#pragma once

#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "hri_tools/HRI.hpp"

class Listen
{
public:
  using ExecuteTask =
    executor_interfaces::action::ExecuteTask;

  using GoalHandle =
    rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Listen(const rclcpp::Node::SharedPtr& node,
         const std::shared_ptr<HRI>& hri);

  bool run(
    const std::string& grammar,
    const std::shared_ptr<GoalHandle>& goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<HRI> hri;
  Executor executor;
};

