#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "manipulation_tools/Manipulation.hpp"
#include "hardware_tools/Hardware.hpp"

// ─────────────────────────────────────────────────────────────
//   Take
// ─────────────────────────────────────────────────────────────
enum class TakeState
{
    READY,
    PLAN_REACH,
    EXECUTE_REACH,
    CLOSE_GRIPPER,
    PLAN_RETRACT,
    EXECUTE_RETRACT,
    SUCCESS,
    FAILED
};

class Take
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Take(
        const rclcpp::Node::SharedPtr&        node,
        const std::shared_ptr<Manipulation>&  manipulation,
        const std::shared_ptr<Hardware>&      hardware);

    Result run(
        const std::string&                  arm,
        const geometry_msgs::msg::PoseStamped& target_pose,
        const std::shared_ptr<GoalHandle>&  gh);

private:

    // ── helpers ──────────────────────────────────────────────

    bool attempt(
        const std::string& step,
        const std::shared_ptr<GoalHandle>& gh,
        std::function<bool(std::string&)> fn);

    geometry_msgs::msg::PoseStamped retractPose(
        const std::string& arm) const;

    void publishFeedback(
        const std::shared_ptr<GoalHandle>& gh,
        TakeState state,
        const std::string& detail = "");

    static std::string stateName(TakeState s);


    rclcpp::Node::SharedPtr       node_;
    std::shared_ptr<Manipulation> manipulation_;
    std::shared_ptr<Hardware>     hardware_;

    static constexpr int MAX_ATTEMPTS = 2;
    std::vector<std::string> joint_names_;
    std::vector<double>      waypoints_;
};