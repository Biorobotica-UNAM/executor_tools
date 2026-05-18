#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"
#include "hri_tools/HRI.hpp"

#include <string>
#include <array>

// =========================================================
// Estados de la FSM para Give
// =========================================================
enum class GiveState {
    READY,
    ALIGNING,
    MOVING_ARM,
    SPEAKING,
    WAITING,
    SUCCESS,
    FAILED
};

class Give
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Give(const rclcpp::Node::SharedPtr& node,
         const std::shared_ptr<Hardware>& hardware,
         const std::shared_ptr<HRI>& hri);

    Result run(const std::string& argument,
               const std::shared_ptr<GoalHandle>& gh);

private:
    Result enable(const std::shared_ptr<GoalHandle>& gh);
    Result disable(const std::shared_ptr<GoalHandle>& gh);

    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         GiveState state,
                         const std::string& detail = "");

    static std::string stateName(GiveState s);

    // Mismas constantes cinemáticas heredadas de Point
    static constexpr std::array<double, 6> ARM_POINTING = {0.6, 0.1, 0.3, 1.2, 0.0, 0.0};
    static constexpr std::array<double, 6> ARM_RESTING  = {0.0, 0.0, 0.3, 1.5, 0.0, 1.2};

    static constexpr double ANGULAR_SPEED = 0.6;   // rad/s
    static constexpr double ALIGN_EXTRA   = 0.3;   // s

    rclcpp::Node::SharedPtr   node_;
    std::shared_ptr<Hardware> hardware_;
    std::shared_ptr<HRI>      hri_;
};