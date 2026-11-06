#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"

#include <memory>
#include <string>

enum class TestHeadState {
    READY,
    LOOKING_LEFT,
    LOOKING_RIGHT,
    CENTERING_HORIZONTAL,
    LOOKING_UP,
    LOOKING_DOWN,
    CENTERING_VERTICAL,
    SUCCESS,
    FAILED
};

class TestHead
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    TestHead(const rclcpp::Node::SharedPtr&     node,
             const std::shared_ptr<Hardware>&   hardware);

    Result run(const std::string&                 argument,
               const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         TestHeadState                      state,
                         const std::string&                 detail = "");

    static std::string stateName(TestHeadState s);

    rclcpp::Node::SharedPtr   node_;
    std::shared_ptr<Hardware> hardware_;
};
