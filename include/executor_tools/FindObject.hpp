#pragma once

#include <memory>
#include <string>
#include <vector>

#include "executor_interfaces/action/execute_task.hpp"

#include "executor_tools/Result.hpp"

#include "hardware_tools/Hardware.hpp"
#include "vision_tools/Vision.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class FindObject
{
public:

    using ExecuteTask = executor_interfaces::action::ExecuteTask;

    using GoalHandle =
        rclcpp_action::ServerGoalHandle<ExecuteTask>;

    enum class FindObjectState
    {
        READY,
        MOVING_HEAD,
        STABILIZING,
        DETECTING,
        LOOKING_AT_TARGET,
        SUCCESS,
        FAILED
    };

    FindObject(
        const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<Hardware>& hardware,
        const std::shared_ptr<Vision>& vision);

    Result run(
        const std::string& object_name,
        const std::shared_ptr<GoalHandle>& gh);

private:

    bool detectWithOWL(
        const std::string& object_name,
        geometry_msgs::msg::PoseStamped& pose);

    bool detectEmptyChair(
        geometry_msgs::msg::PointStamped& point);

    void publishFeedback(
        const std::shared_ptr<GoalHandle>& gh,
        FindObjectState state,
        const std::string& detail = "");

    std::string stateName(FindObjectState s);

private:

    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<Hardware> hardware_;

    std::shared_ptr<Vision> vision_;

    std::vector<std::pair<float,float>> scan_poses_ =
    {
        { 0.0f,  -0.35f},
        {-0.35f, -0.45f},
        { 0.35f, -0.45f},
        { 0.0f,  -0.60f},
        {-0.25f, -0.65f},
        { 0.25f, -0.65f}
    };
};
