#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"

#include "hardware_tools/Hardware.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "vision_tools/Vision.hpp"

#include <memory>
#include <string>
#include <vector>


enum class AnalyzeObjectsState {
  READY,
  PARSING_ARGS,
  CALLING_SERVICE,
  UPDATING_KNOWLEDGE,
  SUCCESS,
  FAILED
};

class AnalyzeObjects {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  AnalyzeObjects(const rclcpp::Node::SharedPtr &node,
                 const std::shared_ptr<Hardware> &hardware,
                 const std::shared_ptr<Vision> &vision,
                 const std::shared_ptr<Knowledge> &knowledge);

  Result run(const std::string &argument,
             const std::shared_ptr<GoalHandle> &gh);

private:
  // Métodos internos que consumen directamente tu fachada de Vision.hpp
  bool runCount(const std::string &target_class, bool reset_count, std::string &out_msg);
  bool runSize(const std::string &target_class, const std::string &filter_mode, int32_t limit, std::string &out_msg);
  bool runColor(const std::string &target_class, const std::string &color, std::string &out_msg);
  bool runCompare(const std::vector<std::string> &classes, std::string &out_msg);

  // Publicación de feedback usando el enum del analizador
  void publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                       AnalyzeObjectsState state, const std::string &detail = "");

  std::string stateName(AnalyzeObjectsState s);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Hardware> hardware_;
  std::shared_ptr<Vision> vision_;
  std::shared_ptr<Knowledge> knowledge_;
};