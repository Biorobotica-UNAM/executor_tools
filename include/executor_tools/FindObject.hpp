#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "tasks_tools/Tasks.hpp"
#include "executor_tools/Knowledge.hpp"
#include <string>
#include <memory>

class FindObject 
{
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  FindObject(const rclcpp::Node::SharedPtr& node,
             const std::shared_ptr<Tasks>& tasks,
             const std::shared_ptr<Knowledge>& knowledge);

  /**
   * @brief Ejecuta el escaneo de cabeza para encontrar un objeto.
   * @param object_label Etiqueta o nombre directo del objeto (ej. "drink" o "apple").
   */
  bool run(const std::string& object_label,
           const std::shared_ptr<GoalHandle>& goalHandle);

private:
  std::string resolveObjectName(const std::string& label);

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Tasks> tasks;
  std::shared_ptr<Knowledge> knowledge;
  Executor executor;
};