#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "tasks_tools/Tasks.hpp"
#include "executor_tools/Knowledge.hpp"
#include <string>
#include <memory>

class FindWaving 
{
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  FindWaving(const rclcpp::Node::SharedPtr& node,
             const std::shared_ptr<Tasks>& tasks,
             const std::shared_ptr<Knowledge>& knowledge);

  /**
   * @brief Ejecuta la búsqueda de una persona agitando la mano (waving).
   * @param argument Argumento opcional (ej. "human").
   */
  bool run(const std::string& argument,
           const std::shared_ptr<GoalHandle>& goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Tasks> tasks;
  std::shared_ptr<Knowledge> knowledge;
  Executor executor;
};