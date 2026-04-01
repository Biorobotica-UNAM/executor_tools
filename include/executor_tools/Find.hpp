#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "tasks_tools/Tasks.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include <utility>
#include <string>

class Find 
{
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Find(const rclcpp::Node::SharedPtr& node,
       const std::shared_ptr<Tasks>& tasks,
       const std::shared_ptr<Knowledge>& knowledge);

  bool run(const std::string& category_item,
           const std::shared_ptr<GoalHandle>& goalHandle);

private:
  std::pair<std::string, std::string> parseCategoryItem(const std::string& category_item);
  std::string resolveItem(const std::string& item);

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Tasks> tasks;
  std::shared_ptr<Knowledge> knowledge;
  Executor executor;
};
