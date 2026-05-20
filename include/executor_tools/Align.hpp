#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"

#include "hardware_tools/Hardware.hpp"
#include "vision_tools/Vision.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <string>
#include <memory>

enum class AlignState {
    READY,
    MOVING_HEAD,
    MOVING_TORSO,
    GET_ALIGNMENT,
    ROTATING,
    STABILIZING,
    SUCCESS,
    FAILED
};

class Align
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Align(
        const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<Hardware>& hardware,
        const std::shared_ptr<Vision>& vision);

    Result run(
        const std::string& argument,
        const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(
        const std::shared_ptr<GoalHandle>& gh,
        AlignState state,
        const std::string& detail = "");

    static std::string stateName(AlignState s);

    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<Hardware> hardware_;
    std::shared_ptr<Vision>   vision_;
};
