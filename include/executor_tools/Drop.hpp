#pragma once

#include <memory>
#include <string>

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

enum class DropState {
  READY,
  WAIT_DROP_TF,
  VALIDATE_DROP_TF,
  SUCCESS,
  FAILED
};

// Drop consume una TF ya generada por FindSpace. Por ahora solo valida que esa
// marca exista y este dentro de limites; despues aqui se conectara brazo/gripper
// para ejecutar el soltar fisico.
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
    DropBounds bounds;
  };

  void publishFeedback(const std::shared_ptr<GoalHandle> &gh, DropState state,
                       const std::string &detail = "");

  static DropTarget resolveDropTarget(const std::string &argument);
  static std::string stateName(DropState s);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Hardware> hardware_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
