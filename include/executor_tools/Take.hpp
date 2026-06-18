#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>
#include <optional>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "executor_tools/VisualGraspApproach.hpp"
#include "manipulation_tools/Manipulation.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "hardware_tools/Hardware.hpp"
#include "hri_tools/HRI.hpp"

// ─────────────────────────────────────────────────────────────
//   TakeState
// ─────────────────────────────────────────────────────────────
enum class TakeState
{
    READY,
    GETTING_TARGET,
    VERIFYING_VACANCY,
    PREPARING_EXTEND,
    OPENNING_GRIPPER,
    PLANNING_EXTEND,
    EXECUTING_EXTEND,
	MOVING_FORWARD,
    CLOSING_GRIPPER,
    REPORTING_OCCUPANCY,
	MOVING_BACKWARD,
    PLANNING_RETRACT,
    EXECUTING_RETRACT,
    PREPARING_TRANSPORT,
    SUCCESS,
    FAILED
};

// ─────────────────────────────────────────────────────────────
//   Take
// ─────────────────────────────────────────────────────────────
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
        const std::shared_ptr<Knowledge>&     knowledge,
        const std::shared_ptr<HRI>&           hri);

    Result run(
        const std::string&                 argument,
        const std::shared_ptr<GoalHandle>& gh);

private:
    // ── Grasp-type classification ──────────────────────────────

	inline static const std::unordered_set<std::string> PINCH_OBJECTS = {
		// Cutlery
		"spoon", "fork", "knife", "chopstick", "teaspoon",
		"dessert_spoon", "butter_knife", "spatula", "ladle",
		// Writing / office
		"pen", "pencil", "marker", "straw", "chopsticks",
		// Small flat items
		"card", "coin", "key", "remote",
	};

	inline static const std::unordered_set<std::string> GRAB_OBJECTS = {
		// Tableware
		"cup", "mug", "glass", "bowl", "plate",
		"saucer", "pitcher", "teapot", "kettle",
		// Containers
		"bottle", "can", "jar", "box", "carton",
		"thermos", "flask", "container", "tray",
		// Food items
		"apple", "orange", "banana", "bread",
		// Misc household
		"book", "spray", "soap", "shampoo",
	};

    // ── FSM helpers ───────────────────────────────────────────
    bool parseArgument(
        const std::string& argument,
        std::string& out_name,
        std::string& out_index) const;

    std::optional<geometry_msgs::msg::PoseStamped> jsonToPose(
        const json& data,
        rclcpp::Clock::SharedPtr clock);

    bool selectArm(
        std::string& arm_group,
        std::string& hand,
        const std::string& right_obj,
        const std::string& left_obj,
        double y_coord) const;

    std::string selectTip(const std::string& object_name) const;

    PoseStamped retractPose(const std::string& arm) const;

    void publishFeedback(
        const std::shared_ptr<GoalHandle>& gh,
        TakeState                          state,
        const std::string&                 detail = "");

    static std::string stateName(TakeState s);

    // ── Arm poses ─────────────────────────────────────────────
    static constexpr std::array<double, 6> PREPARE_POSE   = {-0.9, 0.0, 0.0, 1.5, 0.0, 0.9};
    static constexpr std::array<double, 6> TRANSPORT_POSE = {-0.9, 0.0, 0.0, 1.5, 0.0, 0.9};

    // ── Dependencies ──────────────────────────────────────────
    rclcpp::Node::SharedPtr       node_;
    std::shared_ptr<Manipulation> manipulation_;
    std::shared_ptr<Hardware>     hardware_;
    std::shared_ptr<Knowledge>    knowledge_;
    std::shared_ptr<HRI>          hri_;
    std::unique_ptr<VisualGraspApproach> visual_grasp_approach_;

    // ── Per-run state ─────────────────────────────────────────
    std::string              arm_group_;
    std::vector<std::string> joint_names_;
    std::vector<double>      waypoints_;

    // ── Physical limits ───────────────────────────────────────
    double                   x_limit = 0.6;
};
