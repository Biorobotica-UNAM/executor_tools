#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>
#include <memory>

enum class PointState {
    READY,
    ALIGNING_BASE,
    MOVING_HEAD,
    POINTING_ARM,
    SUCCESS,
    FAILED
};

class Point
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    explicit Point(const rclcpp::Node::SharedPtr& node,
                      const std::shared_ptr<Hardware>& hardware);

    Result run(const std::shared_ptr<GoalHandle>& gh);
    Result disable(const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         PointState state,
                         const std::string& detail = "");

    static std::string stateName(PointState s);

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool use_takeshi_;
    std::string camera_frame_;
    std::string base_frame_;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Hardware> hardware_;
};