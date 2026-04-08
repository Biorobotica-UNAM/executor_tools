#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"

#include "vision_tools/Vision.hpp"
#include "executor_tools/Knowledge.hpp"
#include "executor_tools/Executor.hpp"

#include "faces_interfaces/srv/face_recog.hpp"

class Describe {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Describe(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Knowledge> &knowledge);

  bool run(const std::string &person_name,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Knowledge> knowledge;
  
  rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr client;
  
  Executor executor;
};