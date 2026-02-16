#pragma once

#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "hri_tools/HRI.hpp"

#include <string>
#include <memory>

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
    const std::string& action_name,
    const std::shared_ptr<GoalHandle>& goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<HRI> hri;
  Executor executor;

  std::string getRandomPhrase(const std::string& action_name);
  std::string replacePlaceholdersWithSaved(std::string text);
  std::string readSavedValue(const std::string& filename);
};

