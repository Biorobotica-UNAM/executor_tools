#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "executor_tools/Knowledge.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

class Save {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Save(const rclcpp::Node::SharedPtr &node,
       const std::shared_ptr<Knowledge> &knowledge_tools);

  bool run(const std::string &argument,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Knowledge> knowledge;
  Executor executor;
};