#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "faces_interfaces/srv/face_recog.hpp" 

class Find {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  explicit Find(const rclcpp::Node::SharedPtr &node);

  bool run(const std::string &person_name,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  Executor executor;
  
  rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr client;
};