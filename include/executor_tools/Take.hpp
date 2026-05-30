#pragma once

#include <memory>
#include <string>
#include <vector>
#include <optional>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "manipulation_tools/Manipulation.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "hardware_tools/Hardware.hpp"

// ─────────────────────────────────────────────────────────────
//   Take
// ─────────────────────────────────────────────────────────────
enum class TakeState
{
    READY,
    GETTING_TARGET,
    PREPARING_EXTEND,
    PLANNING_EXTEND,
    EXECUTING_EXTEND,
    CLOSING_GRIPPER,
    PLANNING_RETRACT,
    EXECUTING_RETRACT,
    SUCCESS,
    FAILED
};

class Take
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using json        = nlohmann::json;

    Take(
        const rclcpp::Node::SharedPtr&        node,
        const std::shared_ptr<Manipulation>&  manipulation,
        const std::shared_ptr<Hardware>&      hardware,
        const std::shared_ptr<Knowledge>&     knowledge);

    Result run(
        const std::string&                 argument,
        const std::shared_ptr<GoalHandle>& gh);

private:
    bool parseArgument(
        const std::string& argument,
        std::string& out_name,
        std::string& out_index) const;

    std::optional<geometry_msgs::msg::PoseStamped> jsonToPose(
        const json& data,
        rclcpp::Clock::SharedPtr clock);

    PoseStamped retractPose(const std::string& arm) const;

    void publishFeedback(
        const std::shared_ptr<GoalHandle>& gh,
        TakeState                          state,
        const std::string&                 detail = "");

    static std::string stateName(TakeState s);

    static constexpr std::array<double, 6> PREPARE_POSE = {0.0, 0.0, 0.0, 1.5, 0.0, 0.9};

    rclcpp::Node::SharedPtr       node_;
    std::shared_ptr<Manipulation> manipulation_;
    std::shared_ptr<Hardware>     hardware_;
    std::shared_ptr<Knowledge>    knowledge_;

    std::string              arm_group_;
    std::vector<std::string> joint_names_;
    std::vector<double>      waypoints_;
};
