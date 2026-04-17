#pragma once

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "executor_tools/Executor.hpp"

//nodo en hri_tools llamada take y drop

class Take
{
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  explicit Take(const rclcpp::Node::SharedPtr& node);

  bool run(const std::string& argument,
           const std::shared_ptr<GoalHandle>& goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
  Executor executor;
};


/*#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "executor_tools/Knowledge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "take_interfaces/srv/take.hpp" // Interfaz de tu Máquina de Estados

class Take {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Take(const rclcpp::Node::SharedPtr &node,
       const std::shared_ptr<Knowledge> &knowledge);

  bool run(const std::string &object_name,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Knowledge> knowledge;
  Executor executor;
};*/