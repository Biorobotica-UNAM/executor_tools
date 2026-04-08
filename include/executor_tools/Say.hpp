#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "hri_tools/HRI.hpp"
#include "executor_tools/Knowledge.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <random>
#include <regex>

class Say
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Say(const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<HRI>& hri,
        const std::shared_ptr<Knowledge>& knowledge);

    bool run(const std::string& action_name,
             const std::shared_ptr<GoalHandle>& goalHandle);

private:
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<HRI> hri;
    std::shared_ptr<Knowledge> knowledge;
    Executor executor;

    std::string readSavedValue(const std::string& filename);
    std::string replacePlaceholdersWithSaved(const std::string& text);
    std::string getRandomPhrase(const std::string& action_name);
};

