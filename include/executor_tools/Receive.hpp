#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"
#include "hri_tools/HRI.hpp"
#include "knowledge_tools/Knowledge.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <array>

// =========================================================
// Estados de la FSM para Receive
// =========================================================
enum class ReceiveState {
    READY,
    VERIFYING_VACANCY,
    MOVING_ARM,
    OPENING_GRIPPER,
    SPEAKING,
    CLOSING_GRIPPER,
    REPORTING_OCCUPANCY,
    SUCCESS,
    FAILED
};

class Receive
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;
    using json        = nlohmann::json;

    Receive(const rclcpp::Node::SharedPtr& node,
            const std::shared_ptr<Hardware>& hardware,
            const std::shared_ptr<HRI>& hri,
            const std::shared_ptr<Knowledge>& knowledge);

    Result run(const std::string& object_name,
               const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         ReceiveState state,
                         const std::string& detail = "");

    static std::string stateName(ReceiveState s);

    // Constantes cinemáticas heredadas de Point
    static constexpr std::array<double, 6> LEFT_ARM_POINTING = {0.4, 0.0, 0.3, 1.0, 0.0, 0.2};
    static constexpr std::array<double, 6> RIGHT_ARM_POINTING = {0.4, 0.0, -0.3, 1.0, 0.0, 0.2};
    static constexpr std::array<double, 6> ARM_TRANSPORT  = {0.0, 0.0, 0.0, 1.5, 0.0, 1.35};

    rclcpp::Node::SharedPtr     node_;
    std::shared_ptr<Hardware>   hardware_;
    std::shared_ptr<HRI>        hri_;
    std::shared_ptr<Knowledge>  knowledge_;
};
