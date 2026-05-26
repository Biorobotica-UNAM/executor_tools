#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"

#include "hardware_tools/Hardware.hpp"
#include "vision_tools/Vision.hpp"
#include "knowledge_tools/Knowledge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "nlohmann/json.hpp"

#include <string>
#include <vector>

// ============================================================
// State machine for the FindPerson action
// ============================================================

enum class FindPersonState {
    READY,
    SCANNING,
    DETECTING,
    CALCULATING_POSE,
    LOOK_AT_TARGET,
    IDENTIFYING,
    REGISTERING,
    DESCRIBING,
	UPDATING,
    SUCCESS,
    FAILED
};


class FindPerson
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    FindPerson(
		const rclcpp::Node::SharedPtr& node,
		const std::shared_ptr<Hardware>& hardware,
		const std::shared_ptr<Vision>& vision,
		const std::shared_ptr<Knowledge>& knowledge);

    Result run(
        const std::string& argument,
        const std::shared_ptr<GoalHandle>& gh);

private:
    bool scanPosition(
		double pan,
		double tilt,
		bool anybody,
		const std::string& target,
		std::string& found_name,
		const std::shared_ptr<GoalHandle>& gh);

    void publishFeedback(
        const std::shared_ptr<GoalHandle>& gh,
        FindPersonState state,
        const std::string& detail = "");

    static std::string stateName(FindPersonState s);
    static std::string generatePersonName();

	nlohmann::json buildPersonPayload(
		const std::string& name,
		const geometry_msgs::msg::PoseStamped& pose,
		const std::string& description) const;

	std::vector<std::string> splitDescription(
		const std::string& description) const;

    rclcpp::Node::SharedPtr   node_;
    std::shared_ptr<Hardware> hardware_;
    std::shared_ptr<Vision>   vision_;
	std::shared_ptr<Knowledge> knowledge_;
};
