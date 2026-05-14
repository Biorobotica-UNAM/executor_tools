#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hri_tools/HRI.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>
#include <random>
#include <regex>

// =========================================================
// Estados de la FSM
// =========================================================
enum class SayState {
    READY,
    LOADING,
    SPEAKING,
    SUCCESS,
    FAILED
};

class Say
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Say(const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<HRI>&    hri);

    ToolResult run(const std::string&                 phrase_key,
                   const std::shared_ptr<GoalHandle>& gh);

private:
    std::string loadPhrase(const std::string& key);
    std::string resolvePlaceholders(const std::string& text,
                                    const std::string& arg);

    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         SayState                           state,
                         const std::string&                 detail = "");

    static std::string stateName(SayState s);

    // archivos de frases en orden de búsqueda
    static const std::vector<std::string> PHRASE_FILES;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<HRI>    hri_;

    static std::mt19937 rng_;
};
