#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "tasks_tools/Tasks.hpp"
#include "executor_tools/Knowledge.hpp"
#include <string>
#include <memory>

class FindPerson 
{
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  FindPerson(const rclcpp::Node::SharedPtr& node,
             const std::shared_ptr<Tasks>& tasks,
             const std::shared_ptr<Knowledge>& knowledge);

  /**
   * @brief Ejecuta la búsqueda de una persona.
   * @param name_to_find Nombre de la persona o "anybody" para búsqueda genérica.
   */
  bool run(const std::string& name_to_find,
           const std::shared_ptr<GoalHandle>& goalHandle);

private:
  std::string resolvePersonName(const std::string& name);

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Tasks> tasks;
  std::shared_ptr<Knowledge> knowledge;
  Executor executor;
};