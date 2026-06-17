#pragma once

#include <memory>
#include <optional>
#include <string>

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "navigation_tools/Navigation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

enum class SaveLocState {
  READY,
  SAVING,
  SUCCESS,
  FAILED
};

class SaveLoc {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;
  using PoseBuffer =
      std::shared_ptr<std::optional<geometry_msgs::msg::PoseStamped>>;

  SaveLoc(const rclcpp::Node::SharedPtr &node,
          const std::shared_ptr<Navigation> &navigation,
          const PoseBuffer &customer_pose_map = nullptr);

  Result run(const std::string &location_name,
             const std::shared_ptr<GoalHandle> &gh);

private:
  void publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                       SaveLocState state,
                       const std::string &detail = "") const;

  static std::string stateName(SaveLocState state);
  static std::string trim(std::string value);
  static std::string normalize(std::string value);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Navigation> navigation_;
  PoseBuffer customer_pose_map_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};