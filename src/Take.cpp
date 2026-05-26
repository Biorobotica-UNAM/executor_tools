#include "executor_tools/Take.hpp"

#include <chrono>
#include <thread>
#include <functional>

using namespace std::chrono_literals;

// TODO: Choose which arm to use. Currently, left arm hardcoded.

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
Take::Take(
    const rclcpp::Node::SharedPtr&        node,
    const std::shared_ptr<Manipulation>&  manipulation,
    const std::shared_ptr<Hardware>&      hardware,
    const std::shared_ptr<Knowledge>&     knowledge)
: node_(node),
  manipulation_(manipulation),
  hardware_(hardware),
  knowledge_(knowledge)
{}

// ─────────────────────────────────────────────────────────────
//  run
// ─────────────────────────────────────────────────────────────
Result Take::run(
    const std::string&                     argument,
    const std::shared_ptr<GoalHandle>&     gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_inactive"};

    // ── READY ────────────────────────────────────────────
    publishFeedback(gh, TakeState::READY, argument);

    // ── GETTING_TARGET ────────────────────────────────────────────
    publishFeedback(gh, TakeState::GETTING_TARGET);

    std::string category_name;
    std::string object_index;

    if (!parseArgument(
            argument,
            category_name,
            object_index))
    {
        publishFeedback(
            gh,
            TakeState::FAILED,
            "invalid_argument");

        return {
            false,
            "FAILED",
            "invalid_argument"
        };
    }

    json target_data = knowledge_->get_world_state(
                                category_name,
                                object_index, 
                                "manipulation");

    if (target_data.empty())
    {
        publishFeedback(
            gh,
            TakeState::FAILED,
            "invalid_manipulation_data");

        return {
            false,
            "FAILED",
            "invalid_manipulation_data"
        };
    }

    // ── PLAN_REACH ────────────────────────────────────────────
    publishFeedback(gh, TakeState::PLANNING_EXTEND);

    std::string msg;
    PoseStamped target_pose = jsonToPose(target_data, node_->get_clock());

    bool ok = manipulation_->PlanTrajectory(
            target_pose, "left_arm_group", joint_names_, waypoints_, msg);

    if (!ok)
        return {false, "FAILED", msg};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── EXECUTE_REACH ─────────────────────────────────────────
    publishFeedback(gh, TakeState::EXECUTING_EXTEND);

    ok = manipulation_->FollowTrajectory(
            "left_arm_group", joint_names_, waypoints_, msg);

    if (!ok)
        return {false, "FAILED", msg};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── CLOSE_GRIPPER ─────────────────────────────────────────
    publishFeedback(gh, TakeState::CLOSING_GRIPPER);

    ok = hardware_->MoveLeftGripper("close");

    if (!ok) {
        publishFeedback(gh, TakeState::FAILED, "gripper_close_failed");
        return {false, "FAILED", "gripper_close_failed"};
    }

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── PLAN_RETRACT ──────────────────────────────────────────
    publishFeedback(gh, TakeState::PLANNING_RETRACT);

    const auto home = retractPose("left_arm_group");

    ok = manipulation_->PlanTrajectory(
            home, "left_arm_group", joint_names_, waypoints_, msg);

    if (!ok)
        return {false, "FAILED", msg};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── EXECUTE_RETRACT ───────────────────────────────────────
    publishFeedback(gh, TakeState::EXECUTING_RETRACT);

    ok = manipulation_->FollowTrajectory(
            "left_arm_group", joint_names_, waypoints_, msg);

    if (!ok)
        return {false, "FAILED", msg};

    // ── SUCCESS ───────────────────────────────────────────────
    publishFeedback(gh, TakeState::SUCCESS, "left_arm_group");
    return {true, "SUCCESS", std::string("left_arm_group") + "_take_completed"};
}

// ─────────────────────────────────────────────────────────────
//  parseArgument
// ─────────────────────────────────────────────────────────────

bool Take::parseArgument(
    const std::string& argument,
    std::string& out_name,
    std::string& out_index) const
{
    const std::size_t sep =
        argument.rfind('_');

    if (sep == std::string::npos ||
        sep == 0 ||
        sep + 1 == argument.size())
    {
        return false;
    }

    out_name =
        argument.substr(0, sep);

    out_index =
        argument.substr(sep + 1);

    return true;
}

geometry_msgs::msg::PoseStamped Take::jsonToPose(
    const json& data,
    rclcpp::Clock::SharedPtr clock)
{
    PoseStamped pose;

    const auto& p = data["manipulation"];

    pose.header.stamp = clock->now();
    pose.header.frame_id = "map";

    pose.pose.position.x = p[0];
    pose.pose.position.y = p[1];
    pose.pose.position.z = p[2];

    pose.pose.orientation.x = p[3];
    pose.pose.orientation.y = p[4];
    pose.pose.orientation.z = p[5];
    pose.pose.orientation.w = p[6];

    return pose;
}

// ─────────────────────────────────────────────────────────────
//  retractPose
// ─────────────────────────────────────────────────────────────
geometry_msgs::msg::PoseStamped Take::retractPose(
    const std::string& arm) const
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id    = "base_link";
    pose.pose.position.x    =  0.4;
    pose.pose.position.y    = (arm == "right_arm_group") ? -0.35 : 0.35;
    pose.pose.position.z    =  0.9;
    pose.pose.orientation.x =  0.0;
    pose.pose.orientation.y = -0.707;
    pose.pose.orientation.z =  0.0;
    pose.pose.orientation.w =  0.707;
    return pose;
}

// ─────────────────────────────────────────────────────────────
//  publishFeedback
// ─────────────────────────────────────────────────────────────
void Take::publishFeedback(
    const std::shared_ptr<GoalHandle>& gh,
    TakeState                          state,
    const std::string&                 detail)
{
    if (!gh->is_active())
        return;

    std::string msg = "[take] " + stateName(state);

    if (!detail.empty())
        msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// ─────────────────────────────────────────────────────────────
//  stateName
// ─────────────────────────────────────────────────────────────
std::string Take::stateName(TakeState s)
{
    switch (s)
    {
        case TakeState::READY:             return "READY";
        case TakeState::GETTING_TARGET:    return "GETTING_TARGET";
        case TakeState::PLANNING_EXTEND:   return "PLANNING_EXTEND";
        case TakeState::EXECUTING_EXTEND:  return "EXECUTING_EXTEND";
        case TakeState::CLOSING_GRIPPER:   return "CLOSING_GRIPPER";
        case TakeState::PLANNING_RETRACT:  return "PLANNING_RETRACT";
        case TakeState::EXECUTING_RETRACT: return "EXECUTING_RETRACT";
        case TakeState::SUCCESS:           return "SUCCESS";
        case TakeState::FAILED:            return "FAILED";
        default:                           return "UNKNOWN";
    }
}