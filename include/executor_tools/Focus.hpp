#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "vision_tools/Vision.hpp"

#include <string>

enum class FocusState {
    READY,
    FOCUSING,
    SUCCESS,
    FAILED
};

class Focus
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Focus(const rclcpp::Node::SharedPtr& node,
            const std::shared_ptr<Vision>&  vision);

    Result run(const std::string& argument,
               const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         FocusState state,
                         const std::string& detail = "");

    static std::string stateName(FocusState s);

    bool parseArgument(const std::string& argument);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Vision> vision_;
};