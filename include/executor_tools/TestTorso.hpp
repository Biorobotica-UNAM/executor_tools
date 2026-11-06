#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"
#include "hri_tools/HRI.hpp"

#include <memory>
#include <string>

enum class TestTorsoState {
    READY,
    MOVING_MIDDLE,
    MOVING_BOTTOM,
    MOVING_TOP,
    SUCCESS,
    FAILED
};

class TestTorso
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    TestTorso(const rclcpp::Node::SharedPtr&    node,
              const std::shared_ptr<Hardware>& hardware,
              const std::shared_ptr<HRI>&      hri);

    Result run(const std::string&                  argument,
               const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         TestTorsoState                     state,
                         const std::string&                 detail = "");

    static std::string stateName(TestTorsoState s);

    // Reproduce texto por TTS en un hilo desacoplado
    void sayAsync(const std::string& text);

    rclcpp::Node::SharedPtr   node_;
    std::shared_ptr<Hardware> hardware_;
    std::shared_ptr<HRI>      hri_;
};