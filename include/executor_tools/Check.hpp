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
enum class CheckState {
    READY,
    CHECKING,
    SUCCESS,
    FAILED
};

class Check
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Check(const rclcpp::Node::SharedPtr&    node,
          const std::shared_ptr<Hardware>&  hardware);

    Result run(const std::string&                 argument,
               const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         CheckState                         state,
                         const std::string&                 detail = "");

    static std::string stateName(CheckState s);

    rclcpp::Node::SharedPtr   node_;
    std::shared_ptr<Hardware> hardware_;
};
