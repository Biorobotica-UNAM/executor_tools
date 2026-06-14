#include "executor_tools/Take.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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

    // Fetch object name for grasp-type selection
    json object_name_json = knowledge_->get_world_state(
                                category_name, object_index, "name");

    const std::string object_name = object_name_json.is_string()
                                        ? object_name_json.get<std::string>()
                                        : "";

    if (object_name.empty())
        RCLCPP_WARN(node_->get_logger(),
            "Object name not found for '%s_%s' — defaulting to grab tip.",
            category_name.c_str(), object_index.c_str());
    else
        RCLCPP_INFO(node_->get_logger(),
            "Object name: '%s'", object_name.c_str());

    RCLCPP_INFO(node_->get_logger(),
        "Pose JSON: %s", target_data.dump().c_str());

    // ── VERIFYING_VACANCY ─────────────────────────────────────
    publishFeedback(gh, TakeState::VERIFYING_VACANCY);

    json left_object  = knowledge_->get_world_state("robot", "", "left_hand");
    json right_object = knowledge_->get_world_state("robot", "", "right_hand");

    if (left_object.empty() || right_object.empty())
    {
        publishFeedback(gh, TakeState::FAILED, "invalid_vacancy_data");
        return {false, "FAILED", "invalid_vacancy_data"};
    }

    const std::string target_id = category_name + "_" + object_index;
    const std::string left_val  = left_object.get<std::string>();
    const std::string right_val = right_object.get<std::string>();

    if (left_val == target_id || right_val == target_id)
    {
        arm_group_ = (left_val == target_id) ? "left_arm_group" : "right_arm_group";

        publishFeedback(gh, TakeState::VERIFYING_VACANCY, "already_holding_object");

        // ── PLANNING_RETRACT (early path) ─────────────────────
        publishFeedback(gh, TakeState::PLANNING_RETRACT);

        const auto home = retractPose(arm_group_);

        if (!hri_->Say("Already holding, retracting"))
            publishFeedback(gh, TakeState::PLANNING_RETRACT, "say_failed");

        std::string msg;
        bool ok = manipulation_->PlanTrajectory(
                        home, arm_group_, selectTip(object_name),
                        joint_names_, waypoints_, msg);

        if (!ok)
            return {false, "FAILED", msg};

        // ── EXECUTING_RETRACT (early path) ────────────────────
        publishFeedback(gh, TakeState::EXECUTING_RETRACT);

        ok = manipulation_->FollowTrajectory(
                    arm_group_, joint_names_, waypoints_, msg);
        if (!ok)
            return {false, "FAILED", msg};

        // ── PREPARING_TRANSPORT (early path) ──────────────────
        publishFeedback(gh, TakeState::PREPARING_TRANSPORT);

        bool torso_ok = hardware_->MoveTorso(0.3);
        bool arm_ok   = (arm_group_ == "left_arm_group")
            ? hardware_->MoveLeftArm({
                TRANSPORT_POSE[0], TRANSPORT_POSE[1], TRANSPORT_POSE[2],
                TRANSPORT_POSE[3], TRANSPORT_POSE[4], TRANSPORT_POSE[5]})
            : hardware_->MoveRightArm({
                TRANSPORT_POSE[0], TRANSPORT_POSE[1], TRANSPORT_POSE[2],
                TRANSPORT_POSE[3], TRANSPORT_POSE[4], TRANSPORT_POSE[5]});

        if (!arm_ok || !torso_ok)
            return {false, "FAILED", "transport_pose_failed"};

        if (!gh->is_active())
            return {false, "FAILED", "goal_cancelled"};

        // ── SUCCESS (early path) ──────────────────────────────
        publishFeedback(gh, TakeState::SUCCESS, arm_group_);
        return {true, "SUCCESS", arm_group_ + "_already_holding"};
    }

    // ── PREPARING_EXTEND ──────────────────────────────────────
    publishFeedback(gh, TakeState::PREPARING_EXTEND);

    const double y_coord = target_data[1].get<double>();

    if (!selectArm(arm_group_, hand,
                   right_object.get<std::string>(),
                   left_object.get<std::string>(),
                   y_coord))
    {
        publishFeedback(gh, TakeState::FAILED, "arm_not_available");
        return {false, "FAILED", "arm_not_available"};
    }

    // selectTip is now valid — arm_group_ is set
    const std::string tip = selectTip(object_name);

    RCLCPP_INFO(node_->get_logger(),
        "Selected arm: %s | tip: %s | target Y = %.3f",
        arm_group_.c_str(), tip.c_str(), y_coord);

    if (!hri_->Say("Preparing take"))
        publishFeedback(gh, TakeState::PREPARING_EXTEND, "say_failed");

    bool prepare_torso_ok = hardware_->MoveTorso(0.3);
    bool prepare_arm_ok   = (arm_group_ == "left_arm_group")
        ? hardware_->MoveLeftArm({
            PREPARE_POSE[0], PREPARE_POSE[1], PREPARE_POSE[2],
            PREPARE_POSE[3], PREPARE_POSE[4], PREPARE_POSE[5]})
        : hardware_->MoveRightArm({
            PREPARE_POSE[0], PREPARE_POSE[1], PREPARE_POSE[2],
            PREPARE_POSE[3], PREPARE_POSE[4], PREPARE_POSE[5]});

    if (!prepare_arm_ok || !prepare_torso_ok)
    {
        publishFeedback(gh, TakeState::FAILED, "invalid_preparation_pose");
        return {false, "FAILED", "invalid_preparation_pose"};
    }

    // ── OPENNING_GRIPPER ──────────────────────────────────────
    publishFeedback(gh, TakeState::OPENNING_GRIPPER);

    /*bool gripper_ok = (arm_group_ == "left_arm_group")
        ? hardware_->MoveLeftGripper("open")
        : hardware_->MoveRightGripper("open");

    if (!gripper_ok)
    {
        publishFeedback(gh, TakeState::OPENNING_GRIPPER, "gripper_open_failed");
        return {false, "FAILED", "gripper_open_failed"};
    }*/

    // ── PLANNING_EXTEND ───────────────────────────────────────
    publishFeedback(gh, TakeState::PLANNING_EXTEND);

    std::string msg;
    auto pose_opt = jsonToPose(target_data, node_->get_clock());

    if (!pose_opt)
    {
        publishFeedback(gh, TakeState::FAILED, "invalid_manipulation_pose");
        return {false, "FAILED", "invalid_manipulation_pose"};
    }

    auto object_exclusion_pose = pose_opt.value();
    auto target_pose = object_exclusion_pose;

    RCLCPP_INFO(node_->get_logger(),
        "PoseStamped:\n"
        "  frame_id: %s\n"
        "  position:    [%.3f, %.3f, %.3f]\n"
        "  orientation: [%.3f, %.3f, %.3f, %.3f]",
        target_pose.header.frame_id.c_str(),
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z,
        target_pose.pose.orientation.x,
        target_pose.pose.orientation.y,
        target_pose.pose.orientation.z,
        target_pose.pose.orientation.w);

    target_pose.pose.position.x += 0.05;
    // target_pose.pose.position.z += 0.08; 

    RCLCPP_INFO(node_->get_logger(),
        "PoseStamped:\n"
        "  frame_id: %s\n"
        "  position:    [%.3f, %.3f, %.3f]\n"
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

    // Durante el acercamiento al objeto, el propio objeto aparece en el octomap
    // de MoveIt como obstaculo. Excluimos solo una zona local alrededor del
    // centro detectado para conservar mesa y otros obstaculos.
    bool ok = manipulation_->PlanTrajectory(
                    target_pose, arm_group_, tip,
                    joint_names_, waypoints_, msg,
                    true, object_exclusion_pose, 0.14, 0.24);

    if (!ok)
        return {false, "FAILED", msg};

    // La pose final ya contiene la orientacion correcta de la mano. Para evitar
    // que la muneca rote hasta el final, cuando ya esta cerca del objeto,
    // fijamos su valor final desde el primer bloque de la trayectoria.
    prealignWristForApproach(joint_names_, waypoints_);

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

    /* gripper_ok = (arm_group_ == "left_arm_group")
        ? hardware_->MoveLeftGripper("close")
        : hardware_->MoveRightGripper("close");

    if (!gripper_ok)
    {
        publishFeedback(gh, TakeState::FAILED, "gripper_close_failed");
        return {false, "FAILED", "gripper_close_failed"};
    }

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};
    */

    // ── REPORTING_OCCUPANCY ───────────────────────────────────
    publishFeedback(gh, TakeState::REPORTING_OCCUPANCY);

    bool report_ok = knowledge_->update_world_state(
                        "robot", "",
                        hand, category_name + "_" + object_index);

    if (!report_ok)
    {
        publishFeedback(gh, TakeState::FAILED, "report_object_failed");
        return {false, "FAILED", "report_object_failed"};
    }

    // ── PLANNING_RETRACT ──────────────────────────────────────
    publishFeedback(gh, TakeState::PLANNING_RETRACT);

    const auto home = retractPose(arm_group_);

    if (!hri_->Say("Planning retract"))
        publishFeedback(gh, TakeState::PLANNING_RETRACT, "say_failed");

    // Al cerrar la mano, el objeto agarrado y parte del gripper pueden seguir
    // apareciendo en el octomap. Si no se excluyen durante la retraccion,
    // MoveIt puede abortar por colision en el estado inicial antes de planear.
    auto retract_exclusion_pose = target_pose;
    RCLCPP_INFO(node_->get_logger(),
        "Planning retract with carried-object exclusion center=[%.3f %.3f %.3f] radius_xy=%.3f height_z=%.3f",
        retract_exclusion_pose.pose.position.x,
        retract_exclusion_pose.pose.position.y,
        retract_exclusion_pose.pose.position.z,
        0.26,
        0.40);

    ok = manipulation_->PlanTrajectory(
                home, arm_group_, tip,
                joint_names_, waypoints_, msg,
                true, retract_exclusion_pose, 0.26, 0.40);

    if (!ok)
    {
        RCLCPP_WARN(node_->get_logger(),
            "Retract planning failed with primary exclusion (%s). Retrying with wider carried-object exclusion radius_xy=%.3f height_z=%.3f",
            msg.c_str(),
            0.34,
            0.50);

        ok = manipulation_->PlanTrajectory(
                    home, arm_group_, tip,
                    joint_names_, waypoints_, msg,
                    true, retract_exclusion_pose, 0.34, 0.50);
    }

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

    // ── PREPARING_TRANSPORT ───────────────────────────────────
    publishFeedback(gh, TakeState::PREPARING_TRANSPORT);

    bool torso_ok = hardware_->MoveTorso(0.3);
    bool arm_ok   = (arm_group_ == "left_arm_group")
        ? hardware_->MoveLeftArm({
            TRANSPORT_POSE[0], TRANSPORT_POSE[1], TRANSPORT_POSE[2],
            TRANSPORT_POSE[3], TRANSPORT_POSE[4], TRANSPORT_POSE[5]})
        : hardware_->MoveRightArm({
            TRANSPORT_POSE[0], TRANSPORT_POSE[1], TRANSPORT_POSE[2],
            TRANSPORT_POSE[3], TRANSPORT_POSE[4], TRANSPORT_POSE[5]});

    if (!torso_ok || !arm_ok)
        return {false, "FAILED", "transport_pose_failed"};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

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

    if (sep == std::string::npos || sep == 0 || sep + 1 == argument.size())
        return false;

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
    if (!data.is_array() || data.size() != 7)
        return std::nullopt;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp    = clock->now();
    pose.header.frame_id = "base_link";

    pose.pose.position.x    = data[0].get<double>();
    pose.pose.position.y    = data[1].get<double>();
    pose.pose.position.z    = data[2].get<double>();
    pose.pose.orientation.x = data[3].get<double>();
    pose.pose.orientation.y = data[4].get<double>();
    pose.pose.orientation.z = data[5].get<double>();
    pose.pose.orientation.w = data[6].get<double>();

    return pose;
}

// ─────────────────────────────────────────────────────────────
//  selectArm
// ─────────────────────────────────────────────────────────────
bool Take::selectArm(
    std::string& arm_group,
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
//  selectTip
// ─────────────────────────────────────────────────────────────
std::string Take::selectTip(const std::string& object_name) const
{
    const bool is_pinch = PINCH_OBJECTS.count(object_name) > 0;

    if (arm_group_ == "left_arm_group")
        return is_pinch ? "left_pinch_object_link" : "left_grab_object_link";
    else
        return is_pinch ? "right_pinch_object_link" : "right_grab_object_link";
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
    pose.pose.position.y    = (arm == "right_arm_group") ? -0.25 : 0.25;
    pose.pose.position.z    =  1.0;
    pose.pose.orientation.x =  0.0;
    pose.pose.orientation.y = -0.707;
    pose.pose.orientation.z =  0.0;
    pose.pose.orientation.w =  0.707;
    return pose;
}

// ─────────────────────────────────────────────────────────────
//  prealignWristForApproach
// ─────────────────────────────────────────────────────────────
bool Take::prealignWristForApproach(
    const std::vector<std::string>& joint_names,
    std::vector<double>& waypoints) const
{
    if (joint_names.empty() || waypoints.empty())
    {
        RCLCPP_WARN(node_->get_logger(),
            "Cannot prealign wrist: empty trajectory.");
        return false;
    }

    const std::size_t block_size = joint_names.size();
    if (waypoints.size() % block_size != 0)
    {
        RCLCPP_WARN(node_->get_logger(),
            "Cannot prealign wrist: %zu waypoint values are not a multiple of %zu joints.",
            waypoints.size(), block_size);
        return false;
    }

    const std::string wrist_joint =
        (arm_group_ == "left_arm_group") ? "l_wrist_joint" : "r_wrist_joint";

    const auto wrist_it =
        std::find(joint_names.begin(), joint_names.end(), wrist_joint);

    if (wrist_it == joint_names.end())
    {
        RCLCPP_WARN(node_->get_logger(),
            "Cannot prealign wrist: joint '%s' is not present in trajectory.",
            wrist_joint.c_str());
        return false;
    }

    const std::size_t wrist_index =
        static_cast<std::size_t>(std::distance(joint_names.begin(), wrist_it));
    const std::size_t blocks = waypoints.size() / block_size;
    const double initial_wrist = waypoints[wrist_index];
    const double final_wrist = waypoints[(blocks - 1) * block_size + wrist_index];
    const double delta = final_wrist - initial_wrist;

    if (std::abs(delta) < 0.01)
    {
        RCLCPP_INFO(node_->get_logger(),
            "Wrist prealignment skipped: %s already near final value %.3f rad.",
            wrist_joint.c_str(), final_wrist);
        return true;
    }

    for (std::size_t block = 0; block < blocks; ++block)
        waypoints[block * block_size + wrist_index] = final_wrist;

    RCLCPP_INFO(node_->get_logger(),
        "Wrist prealignment applied before approach: joint=%s initial=%.3f final=%.3f delta=%.3f blocks=%zu.",
        wrist_joint.c_str(), initial_wrist, final_wrist, delta, blocks);

    return true;
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
        case TakeState::READY:               return "READY";
        case TakeState::GETTING_TARGET:      return "GETTING_TARGET";
        case TakeState::VERIFYING_VACANCY:   return "VERIFYING_VACANCY";
        case TakeState::PREPARING_EXTEND:    return "PREPARING_EXTEND";
        case TakeState::OPENNING_GRIPPER:    return "OPENNING_GRIPPER";
        case TakeState::PLANNING_EXTEND:     return "PLANNING_EXTEND";
        case TakeState::EXECUTING_EXTEND:    return "EXECUTING_EXTEND";
        case TakeState::CLOSING_GRIPPER:     return "CLOSING_GRIPPER";
        case TakeState::REPORTING_OCCUPANCY: return "REPORTING_OCCUPANCY";
        case TakeState::PLANNING_RETRACT:    return "PLANNING_RETRACT";
        case TakeState::EXECUTING_RETRACT:   return "EXECUTING_RETRACT";
        case TakeState::PREPARING_TRANSPORT: return "PREPARING_TRANSPORT";
        case TakeState::SUCCESS:             return "SUCCESS";
        case TakeState::FAILED:              return "FAILED";
        default:                             return "UNKNOWN";
    }
}
