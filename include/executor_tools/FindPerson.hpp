#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"

#include "hardware_tools/Hardware.hpp"
#include "vision_tools/Vision.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <string>

// ============================================================
// State machine for the FindPerson action
// ============================================================

enum class FindPersonState {
    READY,
    SCANNING,
    DETECTING,
    CALCULATING_POSE,
    LOOK_AT_TARGET,
    IDENTIFYING,
    REGISTERING,
    DESCRIBING,
    SUCCESS,
    FAILED
};


class FindPerson
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    FindPerson(
        const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<Hardware>& hardware,
        const std::shared_ptr<Vision>& vision);

    Result run(
        const std::string& argument,
        const std::shared_ptr<GoalHandle>& gh);

private:
    bool scanPosition(
		double pan,
		double tilt,
		bool anybody,
		const std::string& target,
		std::string& found_name,
		const std::shared_ptr<GoalHandle>& gh);

    void publishFeedback(
        const std::shared_ptr<GoalHandle>& gh,
        FindPersonState state,
        const std::string& detail = "");

    static std::string stateName(FindPersonState s);
    static std::string generatePersonName();

    rclcpp::Node::SharedPtr   node_;
    std::shared_ptr<Hardware> hardware_;
    std::shared_ptr<Vision>   vision_;
};
