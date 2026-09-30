#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "navigation_tools/Navigation.hpp"
#include "hardware_tools/Hardware.hpp"

#include <string>
#include <memory>

// =========================================================
// Estados de la FSM
// =========================================================
enum class GoToState {
    READY,
	MOVING_TORSO,
	MOVING_ARMS,
    NAVIGATING,
    SUCCESS,
    FAILED
};

class GoTo
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    GoTo(const rclcpp::Node::SharedPtr&       node,
		 const std::shared_ptr<Hardware>&     hardware,
         const std::shared_ptr<Navigation>&   navigation);

    Result run(const std::string&                 location_name,
               const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         GoToState                          state,
                         const std::string&                 detail = "");

    static std::string stateName(GoToState s);

	static constexpr std::array<double, 6> LEFT_ARM_NAVIGATING = {-0.9, 0.0, 0.0, 1.5, 0.0, 0.9};
    static constexpr std::array<double, 6> RIGHT_ARM_NAVIGATING = {-0.9, 0.0, 0.0, 1.5, 0.0, 0.9};

    rclcpp::Node::SharedPtr     node_;
	std::shared_ptr<Hardware> hardware_;
    std::shared_ptr<Navigation> navigation_;
};
