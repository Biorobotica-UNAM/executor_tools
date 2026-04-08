#pragma once

#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "hri_tools/HRI.hpp"
#include "executor_tools/Knowledge.hpp"

class Listen
{
public:
  using ExecuteTask =
    executor_interfaces::action::ExecuteTask;

  using GoalHandle =
    rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Listen(const rclcpp::Node::SharedPtr& node,
         const std::shared_ptr<HRI>& hri,
         const std::shared_ptr<Knowledge>& knowledge);

  bool run(
    const std::string& grammar,
    const std::shared_ptr<GoalHandle>& goalHandle);

private:
  std::shared_ptr<Knowledge> knowledge;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<HRI> hri;
  Executor executor;
};

