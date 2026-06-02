#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "hri_tools/HRI.hpp"

#include <string>
#include <array>

// =========================================================
// Estados de la FSM para Give 
// =========================================================
enum class GiveState {
    READY,
    MOVING_ARM,
    SPEAKING,
    MOVING_GRIPPER,
    REPORTING_OCCUPANCY,
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
		 const std::shared_ptr<Knowledge>& knowledge,
         const std::shared_ptr<HRI>& hri);

    Result run(const std::string& object_name,
               const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         GiveState state,
                         const std::string& detail = "");

    static std::string stateName(GiveState s);

    // Constantes cinemáticas del brazo izquierdo (6 DOF)
    static constexpr std::array<double, 6> ARM_POINTING = {0.6, 0.1, 0.3, 1.2, 0.0, 0.0};
    static constexpr std::array<double, 6> ARM_RESTING  = {0.0, 0.0, 0.3, 1.5, 0.0, 1.2};

    rclcpp::Node::SharedPtr   node_;
    std::shared_ptr<Hardware> hardware_;
    std::shared_ptr<HRI>      hri_;
};
