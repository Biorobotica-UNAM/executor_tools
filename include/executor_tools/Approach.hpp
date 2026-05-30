#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vision_tools/Vision.hpp"

#include <memory>
#include <string>

enum class ApproachState {
  READY,
  LOOKING_FOR_TF,
  APPROACHING,
  STABILIZING,
  SUCCESS,
  FAILED
};

class Approach {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Approach(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Hardware> &hardware,
           const std::shared_ptr<Vision> &vision);

  Result run(const std::string &argument,
             const std::shared_ptr<GoalHandle> &gh);

private:
  void publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                       ApproachState state, const std::string &detail = "");

  static std::string stateName(ApproachState s);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Hardware> hardware_;
  std::shared_ptr<Vision> vision_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
