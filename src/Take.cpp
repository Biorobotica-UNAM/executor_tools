#include "executor_tools/Take.hpp"

#include <chrono>
#include <thread>
#include <functional>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
Take::Take(
    const rclcpp::Node::SharedPtr&        node,
    const std::shared_ptr<Manipulation>&  manipulation,
    const std::shared_ptr<Hardware>&      hardware,
    const std::shared_ptr<Knowledge>&     knowledge,
    const std::shared_ptr<HRI>&           hri)
: node_(node),
  manipulation_(manipulation),
  hardware_(hardware),
  knowledge_(knowledge),
  hri_(hri)
{}

// ─────────────────────────────────────────────────────────────
//  run
// ─────────────────────────────────────────────────────────────
Result Take::run(
    const std::string&                 argument,
    const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_inactive"};

    // ── READY ─────────────────────────────────────────────────
    publishFeedback(gh, TakeState::READY, argument);

    // ── GETTING_TARGET ────────────────────────────────────────
    publishFeedback(gh, TakeState::GETTING_TARGET);

    std::string category_name;
    std::string object_index;
    std::string hand;

    if (!parseArgument(argument, category_name, object_index))
    {
        publishFeedback(gh, TakeState::FAILED, "invalid_argument");
        return {false, "FAILED", "invalid_argument"};
    }

    json target_data = knowledge_->get_world_state(
                            category_name, object_index, "manipulation");

    if (target_data.empty())
    {
        publishFeedback(gh, TakeState::FAILED, "invalid_manipulation_data");
        return {false, "FAILED", "invalid_manipulation_data"};
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Pose JSON: %s",
        target_data.dump().c_str());

    // ── VERIFYING_VACANCY ─────────────────────────────────────────
    publishFeedback(gh, TakeState::VERIFYING_VACANCY);

    json left_object = knowledge_->get_world_state(
                                    "robot", "", "left_hand");

    json right_object = knowledge_->get_world_state(
                                    "robot", "", "right_hand");

    // Verify is not an empty field
    if (left_object.empty() || right_object.empty()) {
        publishFeedback(gh, TakeState::FAILED, "invalid_vacancy_data");
        return {false, "FAILED", "invalid_vacancy_data"};
    }

    // TODO: The robot already has the object
    // if (left_object.get<std::string>() == category_name + "_" + object_index)

    // if (right_object.get<std::string>() == category_name + "_" + object_index)

    // ── PREPARING_EXTEND ──────────────────────────────────────────
    publishFeedback(gh, TakeState::PREPARING_EXTEND);

    double y_coord = target_data[1].get<double>();

    if (!selectArm(arm_group_, hand,
                    right_object.get<std::string>(),
                    left_object.get<std::string>(),
                    y_coord)){
        publishFeedback(gh, TakeState::FAILED, "arm_not_available");
        return {false, "FAILED", "arm_not_available"};
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Selected arm: %s (target Y = %.3f)",
        arm_group_.c_str(),
        y_coord);

    if (!hri_->Say("Preparing take"))
        publishFeedback(gh, TakeState::PREPARING_EXTEND, "say_failed");

    bool prepare_ok = (arm_group_ == "left_arm_group")
        ? hardware_->MoveLeftArm(
            {PREPARE_POSE[0], PREPARE_POSE[1], PREPARE_POSE[2],
             PREPARE_POSE[3], PREPARE_POSE[4], PREPARE_POSE[5]})
        : hardware_->MoveRightArm(
            {PREPARE_POSE[0], PREPARE_POSE[1], PREPARE_POSE[2],
             PREPARE_POSE[3], PREPARE_POSE[4], PREPARE_POSE[5]});

    if (!prepare_ok){
        publishFeedback(gh, TakeState::FAILED, "invalid_preparation_pose");
        return {false, "FAILED", "invalid_preparation_pose"};
    }

    // ── OPENNING_GRIPPER ───────────────────────────────────────
    publishFeedback(gh, TakeState::OPENNING_GRIPPER);
    
    bool gripper_ok = (arm_group_ == "left_arm_group")
        ? hardware_->MoveLeftGripper("open") : hardware_->MoveRightGripper("open");

    if (!gripper_ok)
    {
        publishFeedback(gh, TakeState::OPENNING_GRIPPER, "gripper_open_failed");
        return {false, "FAILED", "gripper_open_failed"};
    }

    // ── PLANNING_EXTEND ───────────────────────────────────────
    publishFeedback(gh, TakeState::PLANNING_EXTEND);

    std::string msg;
    auto pose_opt = jsonToPose(target_data, node_->get_clock());

    if (!pose_opt)
    {
        publishFeedback(gh, TakeState::FAILED, "invalid_manipulation_pose");
        return {false, "FAILED", "invalid_manipulation_pose"};
    }

    const auto target_pose = pose_opt.value();

    RCLCPP_INFO(
        node_->get_logger(),
        "PoseStamped:\n"
        "  frame_id: %s\n"
        "  position: [%.3f, %.3f, %.3f]\n"
        "  orientation: [%.3f, %.3f, %.3f, %.3f]",
        target_pose.header.frame_id.c_str(),
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z,
        target_pose.pose.orientation.x,
        target_pose.pose.orientation.y,
        target_pose.pose.orientation.z,
        target_pose.pose.orientation.w);

    if (!hri_->Say("Planning trajectory"))
        publishFeedback(gh, TakeState::PLANNING_EXTEND, "say_failed");

    bool ok = manipulation_->PlanTrajectory(
                    target_pose, arm_group_, joint_names_, waypoints_, msg);

    if (!ok)
        return {false, "FAILED", msg};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── EXECUTING_EXTEND ──────────────────────────────────────
    publishFeedback(gh, TakeState::EXECUTING_EXTEND);

    ok = manipulation_->FollowTrajectory(
                arm_group_, joint_names_, waypoints_, msg);

    if (!ok)
        return {false, "FAILED", msg};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── CLOSING_GRIPPER ───────────────────────────────────────
    publishFeedback(gh, TakeState::CLOSING_GRIPPER);

    gripper_ok = (arm_group_ == "left_arm_group")
         ? hardware_->MoveLeftGripper("close") : hardware_->MoveRightGripper("close");

    if (!gripper_ok)
    {
        publishFeedback(gh, TakeState::FAILED, "gripper_close_failed");
        return {false, "FAILED", "gripper_close_failed"};
    }

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── REPORTING_OCCUPANCY ──────────────────────────────────────
    publishFeedback(gh, TakeState::REPORTING_OCCUPANCY);

    bool report_ok = knowledge_->update_world_state(
                        "robot", "",
                        hand, category_name+"_"+object_index);

    if (!report_ok) {
        publishFeedback(gh, TakeState::FAILED, "report_object_failed");
        return {false, "FAILED", "report_object_failed"};
    }
    
    // ── PLANNING_RETRACT ──────────────────────────────────────
    publishFeedback(gh, TakeState::PLANNING_RETRACT);

    const auto home = retractPose(arm_group_);

    if (!hri_->Say("Planning retract"))
        publishFeedback(gh, TakeState::PLANNING_RETRACT, "say_failed");

    ok = manipulation_->PlanTrajectory(
                home, arm_group_, joint_names_, waypoints_, msg);

    if (!ok)
        return {false, "FAILED", msg};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── EXECUTING_RETRACT ─────────────────────────────────────
    publishFeedback(gh, TakeState::EXECUTING_RETRACT);

    ok = manipulation_->FollowTrajectory(
                arm_group_, joint_names_, waypoints_, msg);

    if (!ok)
        return {false, "FAILED", msg};

    // ── SUCCESS ───────────────────────────────────────────────
    publishFeedback(gh, TakeState::SUCCESS, arm_group_);
    return {true, "SUCCESS", arm_group_ + "_take_completed"};
}

// ─────────────────────────────────────────────────────────────
//  parseArgument
// ─────────────────────────────────────────────────────────────
bool Take::parseArgument(
    const std::string& argument,
    std::string& out_name,
    std::string& out_index) const
{
    const std::size_t sep = argument.rfind('_');

    if (sep == std::string::npos ||
        sep == 0 ||
        sep + 1 == argument.size())
    {
        return false;
    }

    out_name  = argument.substr(0, sep);
    out_index = argument.substr(sep + 1);

    return true;
}

// ─────────────────────────────────────────────────────────────
//  jsonToPose
// ─────────────────────────────────────────────────────────────
std::optional<geometry_msgs::msg::PoseStamped> Take::jsonToPose(
    const json& data,
    rclcpp::Clock::SharedPtr clock)
{
    geometry_msgs::msg::PoseStamped pose;

    if (!data.is_array() || data.size() != 7)
        return std::nullopt;

    pose.header.stamp    = clock->now();
    pose.header.frame_id = "base_link";

    pose.pose.position.x = data[0].get<double>();
    pose.pose.position.y = data[1].get<double>();
    pose.pose.position.z = data[2].get<double>();

    pose.pose.orientation.x = data[3].get<double>();
    pose.pose.orientation.y = data[4].get<double>();
    pose.pose.orientation.z = data[5].get<double>();
    pose.pose.orientation.w = data[6].get<double>();

    return pose;
}

// ─────────────────────────────────────────────────────────────
//  selectArm
// ─────────────────────────────────────────────────────────────
bool Take::selectArm(std::string& arm_group,
                     std::string& hand,
                     const std::string& right_obj,
                     const std::string& left_obj,
                     const double y_coord) const
{
    arm_group = (y_coord >= 0.0) ? "left_arm_group" : "right_arm_group";
    
    if (arm_group == "left_arm_group") {
        hand = "left_hand";
        return (left_obj == "nothing");
    } else {        
        hand = "right_hand";
        return (right_obj == "nothing");
    }
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
        case TakeState::PREPARING_EXTEND:  return "PREPARING_EXTEND";
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
