#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "executor_tools/Knowledge.hpp"

// 1. IMPORTANTE: Forward declaration para que el compilador no se queje
class Vision; 

class MemorizeFace {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  // Solo dejamos el constructor que recibe Vision
  MemorizeFace(const rclcpp::Node::SharedPtr &node, 
               const std::shared_ptr<Vision> &vision,
               const std::shared_ptr<Knowledge> &knowledge);
  bool run(const std::string &person_name,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Vision> vision; 
  std::shared_ptr<Knowledge> knowledge;
  Executor executor;
};