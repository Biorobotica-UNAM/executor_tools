#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "tasks_tools/Tasks.hpp"

#include <utility>
#include <string>

class Find 
{
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Find(const rclcpp::Node::SharedPtr& node,
       const std::shared_ptr<Tasks>& tasks);

  bool run(const std::string& category_item,
           const std::shared_ptr<GoalHandle>& goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Tasks> tasks;
  Executor executor;

  std::pair<std::string, std::string> parseCategoryItem(const std::string& category_item);
};
