#pragma once

#include <memory>
#include <string>

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

enum class DropState {
  READY,
  PREPARE_SENSOR,
  WAIT_DROP_TF,
  VALIDATE_DROP_TF,
  SUCCESS,
  FAILED
};

class Drop {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Drop(const rclcpp::Node::SharedPtr &node,
       const std::shared_ptr<Hardware> &hardware);

  Result run(const std::string &argument,
             const std::shared_ptr<GoalHandle> &gh);

private:
  struct DropBounds {
    double x_min{0.15};
    double x_max{1.20};
    double abs_y_max{0.80};
    double z_min{0.35};
    double z_max{1.40};
    double dist_xy_max{1.40};
  };

  struct DropTarget {
    std::string mode;
    std::string frame;
    std::string service_label;
    double torso_height{0.3};
    double head_pitch{-0.6};
    DropBounds bounds;
  };

  void publishFeedback(const std::shared_ptr<GoalHandle> &gh, DropState state,
                       const std::string &detail = "");

  bool setDropTfNodeEnabled(
      const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr &client,
      const std::string &service_label, bool enabled);

  static DropTarget resolveDropTarget(const std::string &argument);
  static std::string stateName(DropState s);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Hardware> hardware_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      enable_table_drop_tf_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      enable_container_drop_tf_client_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
