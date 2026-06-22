#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "hri_tools/HRI.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <array>

// =========================================================
// Estados de la FSM para Give 
// =========================================================
enum class GiveState {
    READY,
    SELECTING_HAND,
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
    using json        = nlohmann::json;

    Give(const rclcpp::Node::SharedPtr& node,
         const std::shared_ptr<Hardware>& hardware,
		 const std::shared_ptr<Knowledge>& knowledge,
         const std::shared_ptr<HRI>& hri);

    Result run(const std::string& object_name,
               const std::shared_ptr<GoalHandle>& gh);

private:
    // =========================================================
    // selectHand - Search the object in both hands and returns 
    //              the one with it
    // =========================================================
    std::string selectHand(const std::string& obj);

    // =========================================================
    // publishFeedback — Formato anidado fb->feedback.message
    // =========================================================
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         GiveState state,
                         const std::string& detail = "");

    // =========================================================
    // stateName - Converts state number to string
    // =========================================================
    static std::string stateName(GiveState s);

    // Constantes cinemáticas del brazo izquierdo (6 DOF)
    static constexpr std::array<double, 6> LEFT_ARM_POINTING = {0.4, 0.0, 0.3, 1.0, 0.0, 0.2};
    static constexpr std::array<double, 6> RIGHT_ARM_POINTING = {0.4, 0.0, -0.3, 1.0, 0.0, 0.2};
    static constexpr std::array<double, 6> ARM_RESTING  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    rclcpp::Node::SharedPtr    node_;
    std::shared_ptr<Hardware>  hardware_;
	std::shared_ptr<Knowledge> knowledge_;
    std::shared_ptr<HRI>       hri_;
};
