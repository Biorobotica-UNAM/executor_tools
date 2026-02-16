#pragma once

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "hri_tools/HRI.hpp"

class Say
{
public:
  using ExecuteTask =
    executor_interfaces::action::ExecuteTask;

  using GoalHandle =
    rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Say(const rclcpp::Node::SharedPtr& node,
      const std::shared_ptr<HRI>& hri);

  bool run(
    const std::string& text,
    const std::shared_ptr<GoalHandle>& goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<HRI> hri;
  Executor executor;

  std::string getRandomPhrase(
      const std::string& action,
      const std::string& argument);
};

