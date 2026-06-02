#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "navigation_tools/Navigation.hpp"

#include <string>

enum class SavelocState {
    READY,
    SAVING,
    SUCCESS,
    FAILED
};

class Saveloc {
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Saveloc(const rclcpp::Node::SharedPtr&       node,
            const std::shared_ptr<Navigation>&   navigation);

    Result run(const std::string&                 location_name,
               const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         SavelocState                        state,
                         const std::string&                  detail = "");

    static std::string stateName(SavelocState s);

    rclcpp::Node::SharedPtr     node_;
    std::shared_ptr<Navigation> navigation_;
};