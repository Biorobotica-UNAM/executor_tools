#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"

#include "hardware_tools/Hardware.hpp"
#include "vision_tools/Vision.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <string>
#include <vector>
#include <memory> 

enum class FindObjectState {
    READY,
    MOVING_HEAD,
    STABILIZING,
    DETECTING,
    LOOKING_AT_TARGET,
    SUCCESS,
    FAILED
};

class FindObject
{
public:

    using ExecuteTask = executor_interfaces::action::ExecuteTask;

    using GoalHandle =
        rclcpp_action::ServerGoalHandle<ExecuteTask>;

    FindObject(
        const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<Hardware>& hardware,
        const std::shared_ptr<Vision>& vision);

    Result run(
        const std::string& object_name,
        const std::shared_ptr<GoalHandle>& gh);

private:

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

    const std::vector<std::pair<double,double>> scan_poses_ = {
        { 0.0,  -0.35},
        {-0.35, -0.45},
        { 0.35, -0.45},
        { 0.0,  -0.60},
        {-0.25, -0.65},
        { 0.25, -0.65}
    };
};
