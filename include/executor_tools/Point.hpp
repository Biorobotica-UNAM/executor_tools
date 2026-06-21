#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"

#include <string>

// =========================================================
// Estados de la FSM
// =========================================================
enum class PointState {
    READY,
    ALIGNING,       // enable: girando base + centrando cabeza
    POINTING,       // enable: publicando pose de brazo
    RESETTING,      // disable: bajando brazo
    SUCCESS,
    FAILED
};

class Point
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Point(const rclcpp::Node::SharedPtr&   node,
          const std::shared_ptr<Hardware>& hardware);

    Result run(const std::string&                 argument,
               const std::shared_ptr<GoalHandle>& gh);

private:
    Result enable(const std::shared_ptr<GoalHandle>& gh);
    Result disable(const std::shared_ptr<GoalHandle>& gh);

    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         PointState                         state,
                         const std::string&                 detail = "");

    static std::string stateName(PointState s);

    // Posiciones de brazo izquierdo (6 DOF) — Justina
    static constexpr std::array<double, 6> ARM_POINTING = {0.6, 0.1, 0.3, 1.2, 0.0, 0.0};
    static constexpr std::array<double, 6> ARM_RESTING  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    static constexpr double ANGULAR_SPEED = 0.6;   // rad/s

    rclcpp::Node::SharedPtr   node_;
    std::shared_ptr<Hardware> hardware_;
};
