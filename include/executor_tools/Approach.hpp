#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "hardware_tools/Hardware.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vision_tools/Vision.hpp"

#include <memory>
#include <mutex>
#include <string>

#include "rcl_interfaces/srv/set_parameters.hpp"

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

  void pfForceCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void moveRobot(double vx, double vy, double wz);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Hardware> hardware_;
  std::shared_ptr<Vision> vision_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pf_enable_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pf_force_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pf_cmd_vel_pub_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr
      set_params_client_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      global_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_nav_pub_;

  std::mutex force_mutex_;
  double pf_force_x_ = 0.0;
  double pf_force_y_ = 0.0;
};
