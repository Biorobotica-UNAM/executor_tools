#pragma once

#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "hri_tools/HRI.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <random>
#include <memory>
#include <fstream>
#include <iostream>

class Say
{
public:
    using ExecuteTask =
        executor_interfaces::action::ExecuteTask;

    using GoalHandle =
        rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Say(const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<HRI>& hri);

    bool run(const std::string& text,
             const std::shared_ptr<GoalHandle>& goalHandle);

private:
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<HRI> hri;
    Executor executor;

    std::string getRandomPhrase(const std::string& action,
                                const std::string& argument);
    std::string replacePlaceholdersWithSaved(const std::string& text);
    std::string readSavedValue(const std::string& key);
};

