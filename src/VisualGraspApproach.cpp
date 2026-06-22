#include "executor_tools/VisualGraspApproach.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

VisualGraspApproach::VisualGraspApproach(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<Manipulation>& manipulation,
    const std::shared_ptr<Hardware>& hardware)
: node_(node),
  manipulation_(manipulation),
  hardware_(hardware),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
    use_base_final_approach_ =
        declareOrGet<bool>("visual_grasp.use_base_final_approach", true);
    use_cartesian_final_approach_ =
        declareOrGet<bool>("visual_grasp.use_cartesian_final_approach", false);
    use_visual_servo_ =
        declareOrGet<bool>("visual_grasp.use_visual_servo", true);
    base_final_approach_distance_m_ =
        declareOrGet<double>("visual_grasp.base_final_approach_distance_m", 0.0);
    base_final_approach_speed_mps_ =
        declareOrGet<double>("visual_grasp.base_final_approach_speed_mps", 0.05);
    visual_max_attempts_ =
        declareOrGet<int>("visual_grasp.visual_max_attempts", 3);
    visual_min_step_m_ =
        declareOrGet<double>("visual_grasp.visual_min_step_m", 0.005);
    visual_max_step_m_ =
        declareOrGet<double>("visual_grasp.visual_max_step_m", 0.03);
    visual_max_total_correction_m_ =
        declareOrGet<double>("visual_grasp.visual_max_total_correction_m", 0.06);
    front_desired_tcp_object_offset_m_ =
        declareOrGet<double>("visual_grasp.front_desired_tcp_object_offset_m", 0.02);
    front_tolerance_m_ =
        declareOrGet<double>("visual_grasp.front_tolerance_m", 0.01);
    tf_lookup_timeout_sec_ =
        declareOrGet<double>("visual_grasp.tf_lookup_timeout_sec", 0.25);
    right_marker_frame_ =
        declareOrGet<std::string>("visual_grasp.right_marker_frame", "right_gripper_marker");
    left_marker_frame_ =
        declareOrGet<std::string>("visual_grasp.left_marker_frame", "left_gripper_marker");
    right_marker_to_tcp_xyz_ =
        declareOrGet<std::vector<double>>("visual_grasp.right_marker_to_tcp_xyz", {0.0, 0.0, 0.0});
    left_marker_to_tcp_xyz_ =
        declareOrGet<std::vector<double>>("visual_grasp.left_marker_to_tcp_xyz", {0.0, 0.0, 0.0});
}

VisualGraspApproach::PlanResult VisualGraspApproach::planPregrasp(
    const std::string& object_name,
    const geometry_msgs::msg::PoseStamped& object_pose,
    const std::string& arm_group,
    const std::string& tip_link)
{
    PlanResult result;
    result.object_pose = object_pose;

    const auto candidates = candidatesForObject(object_name);
    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
        const auto& candidate = candidates[i];
        if (candidate.mode == GraspMode::TOP && !use_cartesian_final_approach_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Skipping TOP grasp candidate '%s': cartesian final approach is not enabled yet",
                candidate.label.c_str());
            continue;
        }

        auto pregrasp_pose = applyOffset(object_pose, candidate.pregrasp_offset);
        auto grasp_pose = applyOffset(object_pose, candidate.grasp_offset);

        RCLCPP_INFO(
            node_->get_logger(),
            "Trying visual grasp candidate %zu/%zu '%s' mode=%s pregrasp=[%.3f %.3f %.3f] grasp=[%.3f %.3f %.3f]",
            i + 1,
            candidates.size(),
            candidate.label.c_str(),
            candidate.mode == GraspMode::FRONT ? "FRONT" : "TOP",
            pregrasp_pose.pose.position.x,
            pregrasp_pose.pose.position.y,
            pregrasp_pose.pose.position.z,
            grasp_pose.pose.position.x,
            grasp_pose.pose.position.y,
            grasp_pose.pose.position.z);

        std::string msg;
        std::vector<std::string> joint_names;
        std::vector<double> waypoints;
        const bool planned = manipulation_->PlanTrajectory(
            pregrasp_pose,
            arm_group,
            tip_link,
            joint_names,
            waypoints,
            msg,
            true,
            object_pose,
            0.14,
            0.24);

        if (!planned)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Visual grasp candidate failed: '%s' planner_message='%s'",
                candidate.label.c_str(),
                msg.c_str());
            result.message = msg;
            continue;
        }

        orderApproachJointSequence(arm_group, joint_names, waypoints);

        result.success = true;
        result.message = "Pregrasp plan found";
        result.candidate = candidate;
        result.pregrasp_pose = pregrasp_pose;
        result.grasp_pose = grasp_pose;
        result.joint_names = std::move(joint_names);
        result.waypoints = std::move(waypoints);

        RCLCPP_INFO(
            node_->get_logger(),
            "Visual grasp candidate accepted: '%s'",
            candidate.label.c_str());
        return result;
    }

    if (result.message.empty())
        result.message = "No visual grasp candidate could be planned";
    return result;
}

bool VisualGraspApproach::executePregrasp(
    const std::string& arm_group,
    PlanResult& result)
{
    if (!result.success)
        return false;

    std::string msg;
    const bool ok = manipulation_->FollowTrajectory(
        arm_group,
        result.joint_names,
        result.waypoints,
        msg);
    result.message = msg;
    return ok;
}

bool VisualGraspApproach::executeFinalApproach(
    const std::string& arm_group,
    const std::string& tip_link,
    PlanResult& result,
    const ShouldContinue& should_continue)
{
    (void)tip_link;

    if (!result.success)
        return false;

    if (result.candidate.mode == GraspMode::TOP)
    {
        result.message =
            "TOP final approach requires PlanCartesianPath; not implemented yet";
        RCLCPP_WARN(node_->get_logger(), "%s", result.message.c_str());
        return false;
    }

    if (use_visual_servo_ && executeVisualServoFront(arm_group, result, should_continue))
        return true;

    if (!use_base_final_approach_)
    {
        result.message = "Visual servo unavailable and base final approach disabled";
        return false;
    }

    RCLCPP_WARN(
        node_->get_logger(),
        "Visual gripper TF unavailable or insufficient; falling back to fixed base final approach %.3f m",
        base_final_approach_distance_m_);

    if (!moveBase(base_final_approach_distance_m_, should_continue))
    {
        result.message = "goal_cancelled_during_base_final_approach";
        return false;
    }

    result.total_base_advance_m += base_final_approach_distance_m_;
    result.message = "Fixed base final approach completed";
    return true;
}

bool VisualGraspApproach::retreatFinalApproach(
    PlanResult& result,
    const ShouldContinue& should_continue)
{
    if (result.total_base_advance_m <= 0.0)
        return true;

    const double retreat = -result.total_base_advance_m;
    if (!moveBase(retreat, should_continue))
    {
        result.message = "goal_cancelled_during_base_retreat";
        return false;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Retreated base final approach distance %.3f m",
        result.total_base_advance_m);
    result.total_base_advance_m = 0.0;
    return true;
}

std::vector<VisualGraspApproach::Candidate>
VisualGraspApproach::candidatesForObject(const std::string& object_name) const
{
    const auto v = [](double x, double y, double z)
    {
        geometry_msgs::msg::Vector3 value;
        value.x = x;
        value.y = y;
        value.z = z;
        return value;
    };

    std::vector<Candidate> front = {
        {"vision_nominal_front", GraspMode::FRONT, v(0.00, 0.00, 0.00), v(0.00, 0.00, 0.00)},
        {"front_slightly_higher", GraspMode::FRONT, v(0.00, 0.00, 0.005), v(0.00, 0.00, 0.005)},
        {"front_high_clearance", GraspMode::FRONT, v(0.005, 0.00, 0.01), v(0.005, 0.00, 0.01)},
        {"farther_high_clearance", GraspMode::FRONT, v(0.01, 0.00, 0.01), v(0.01, 0.00, 0.01)},
    };

    std::vector<Candidate> top = {
        {"top_high_pregrasp", GraspMode::TOP, v(0.00, 0.00, 0.01), v(0.00, 0.00, 0.01)},
        {"top_medium_pregrasp", GraspMode::TOP, v(0.00, 0.00, 0.02), v(0.00, 0.00, 0.02)},
    };

    if (prefersTopGrasp(object_name))
    {
        top.insert(top.end(), front.begin(), front.end());
        return top;
    }

    front.insert(front.end(), top.begin(), top.end());
    return front;
}

std::vector<VisualGraspApproach::MotorStage>
VisualGraspApproach::approachMotorStages() const
{
    return {
        {"arm", {1, 2, 3, 4}, 1},
        {"wrist", {5, 6}, 1},
        {"torso", {0}, 1},
    };
}

std::size_t VisualGraspApproach::maxRemainingApproachBlocks() const
{
    return 3;
}

std::string VisualGraspApproach::motorJointName(
    const std::string& arm_group,
    std::size_t motor) const
{
    const bool left = arm_group == "left_arm_group";
    switch (motor)
    {
        case 0: return "torso_joint";
        case 1: return left ? "lf_shoulder_joint" : "rf_shoulder_joint";
        case 2: return left ? "ls_shoulder_joint" : "rs_shoulder_joint";
        case 3: return left ? "l_arm_twist_joint" : "r_arm_twist_joint";
        case 4: return left ? "l_elbow_joint" : "r_elbow_joint";
        case 5: return left ? "l_hand_joint" : "r_hand_joint";
        case 6: return left ? "l_wrist_joint" : "r_wrist_joint";
        default: return "";
    }
}

bool VisualGraspApproach::orderApproachJointSequence(
    const std::string& arm_group,
    const std::vector<std::string>& joint_names,
    std::vector<double>& waypoints) const
{
    if (joint_names.empty() || waypoints.empty())
    {
        RCLCPP_WARN(node_->get_logger(),
            "Cannot order approach joints: empty trajectory.");
        return false;
    }

    const std::size_t block_size = joint_names.size();
    if (waypoints.size() % block_size != 0)
    {
        RCLCPP_WARN(node_->get_logger(),
            "Cannot order approach joints: %zu waypoint values are not a multiple of %zu joints.",
            waypoints.size(), block_size);
        return false;
    }

    const std::size_t blocks = waypoints.size() / block_size;
    if (blocks < 2)
    {
        RCLCPP_WARN(node_->get_logger(),
            "Cannot order approach joints: only %zu block(s) available.",
            blocks);
        return false;
    }

    struct ResolvedStage
    {
        std::string label;
        std::vector<std::size_t> joint_indices;
        std::vector<std::string> joint_names;
        std::size_t waypoints{1};
    };

    std::vector<ResolvedStage> resolved_stages;
    for (const auto& stage : approachMotorStages())
    {
        ResolvedStage resolved;
        resolved.label = stage.label;
        resolved.waypoints = stage.waypoints;

        if (resolved.waypoints == 0)
            return false;

        for (const auto motor : stage.motors)
        {
            const std::string joint_name = motorJointName(arm_group, motor);
            const auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
            if (it == joint_names.end())
            {
                RCLCPP_WARN(node_->get_logger(),
                    "Cannot order approach joints: joint '%s' is not present in trajectory.",
                    joint_name.c_str());
                return false;
            }
            resolved.joint_indices.push_back(
                static_cast<std::size_t>(std::distance(joint_names.begin(), it)));
            resolved.joint_names.push_back(joint_name);
        }
        resolved_stages.push_back(resolved);
    }

    const auto initial_first = waypoints.begin();
    const auto final_first = waypoints.begin() + (blocks - 1) * block_size;
    std::vector<double> staged_block(initial_first, initial_first + block_size);
    const std::vector<double> final_block(final_first, final_first + block_size);

    std::size_t configured_stage_waypoints = 0;
    for (const auto& stage : resolved_stages)
        configured_stage_waypoints += stage.waypoints;

    std::vector<double> ordered_waypoints;
    ordered_waypoints.reserve(
        (configured_stage_waypoints + maxRemainingApproachBlocks()) * block_size);

    for (std::size_t i = 0; i < resolved_stages.size(); ++i)
    {
        const auto& stage = resolved_stages[i];
        const auto stage_start_block = staged_block;
        for (std::size_t step = 1; step <= stage.waypoints; ++step)
        {
            const double ratio =
                static_cast<double>(step) / static_cast<double>(stage.waypoints);
            for (const auto joint_index : stage.joint_indices)
            {
                const double start = stage_start_block[joint_index];
                const double goal = final_block[joint_index];
                staged_block[joint_index] = start + ratio * (goal - start);
            }
            ordered_waypoints.insert(
                ordered_waypoints.end(), staged_block.begin(), staged_block.end());
        }

        for (std::size_t j = 0; j < stage.joint_indices.size(); ++j)
        {
            const std::size_t joint_index = stage.joint_indices[j];
            RCLCPP_INFO(node_->get_logger(),
                "Visual grasp joint order stage %zu/%zu: %s joint=%s waypoints=%zu initial=%.3f final=%.3f delta=%.3f",
                i + 1,
                resolved_stages.size(),
                stage.label.c_str(),
                stage.joint_names[j].c_str(),
                stage.waypoints,
                stage_start_block[joint_index],
                final_block[joint_index],
                final_block[joint_index] - stage_start_block[joint_index]);
        }
    }

    const std::size_t remaining_blocks = blocks - 1;
    const std::size_t kept_remaining_blocks =
        std::min(remaining_blocks, maxRemainingApproachBlocks());
    for (std::size_t kept = 1; kept <= kept_remaining_blocks; ++kept)
    {
        std::size_t block = 1;
        if (kept_remaining_blocks > 1)
        {
            block = 1 + ((kept - 1) * (remaining_blocks - 1)) /
                          (kept_remaining_blocks - 1);
        }

        std::vector<double> ordered_block(
            waypoints.begin() + block * block_size,
            waypoints.begin() + (block + 1) * block_size);
        for (const auto& stage : resolved_stages)
            for (const std::size_t joint_index : stage.joint_indices)
                ordered_block[joint_index] = final_block[joint_index];
        ordered_waypoints.insert(
            ordered_waypoints.end(), ordered_block.begin(), ordered_block.end());
    }

    RCLCPP_INFO(node_->get_logger(),
        "Visual grasp trajectory reordered: original_blocks=%zu ordered_blocks=%zu kept_remaining=%zu/%zu",
        blocks,
        ordered_waypoints.size() / block_size,
        kept_remaining_blocks,
        remaining_blocks);

    waypoints = std::move(ordered_waypoints);
    return true;
}

bool VisualGraspApproach::executeVisualServoFront(
    const std::string& arm_group,
    PlanResult& result,
    const ShouldContinue& should_continue)
{
    geometry_msgs::msg::PoseStamped tcp_pose;
    if (!lookupVisualTcp(arm_group, tcp_pose))
        return false;

    double total_correction = 0.0;
    for (int attempt = 0; attempt < visual_max_attempts_; ++attempt)
    {
        if (!should_continue())
            return false;

        if (attempt > 0 && !lookupVisualTcp(arm_group, tcp_pose))
            return total_correction > 0.0;

        const double error_x =
            result.object_pose.pose.position.x -
            tcp_pose.pose.position.x -
            front_desired_tcp_object_offset_m_;

        RCLCPP_INFO(
            node_->get_logger(),
            "Visual FRONT correction attempt %d/%d: object_x=%.3f tcp_x=%.3f desired_offset=%.3f error_x=%.3f",
            attempt + 1,
            visual_max_attempts_,
            result.object_pose.pose.position.x,
            tcp_pose.pose.position.x,
            front_desired_tcp_object_offset_m_,
            error_x);

        if (error_x <= front_tolerance_m_)
        {
            result.message = "Visual front correction within tolerance";
            return true;
        }

        const double remaining_allowed =
            std::max(0.0, visual_max_total_correction_m_ - total_correction);
        if (remaining_allowed <= 0.0)
            break;

        double step = clamp(error_x, visual_min_step_m_, visual_max_step_m_);
        step = std::min(step, remaining_allowed);

        if (!moveBase(step, should_continue))
            return false;

        total_correction += step;
        result.total_base_advance_m += step;
    }

    result.message = total_correction > 0.0
        ? "Visual front correction reached correction limit"
        : "Visual front correction did not move";
    return total_correction > 0.0;
}

bool VisualGraspApproach::lookupVisualTcp(
    const std::string& arm_group,
    geometry_msgs::msg::PoseStamped& tcp_pose) const
{
    const bool left = arm_group == "left_arm_group";
    const std::string& marker_frame = left ? left_marker_frame_ : right_marker_frame_;
    const auto& marker_to_tcp_xyz =
        left ? left_marker_to_tcp_xyz_ : right_marker_to_tcp_xyz_;

    if (marker_frame.empty())
        return false;

    try
    {
        const auto tf = tf_buffer_->lookupTransform(
            "base_link",
            marker_frame,
            tf2::TimePointZero,
            tf2::durationFromSec(std::max(0.0, tf_lookup_timeout_sec_)));

        tf2::Transform base_to_marker;
        tf2::fromMsg(tf.transform, base_to_marker);

        tf2::Transform marker_to_tcp;
        marker_to_tcp.setIdentity();
        if (marker_to_tcp_xyz.size() >= 3)
        {
            marker_to_tcp.setOrigin(tf2::Vector3(
                marker_to_tcp_xyz[0],
                marker_to_tcp_xyz[1],
                marker_to_tcp_xyz[2]));
        }

        const tf2::Transform base_to_tcp = base_to_marker * marker_to_tcp;
        tcp_pose.header.frame_id = "base_link";
        tcp_pose.header.stamp = node_->get_clock()->now();
        tcp_pose.pose.position.x = base_to_tcp.getOrigin().x();
        tcp_pose.pose.position.y = base_to_tcp.getOrigin().y();
        tcp_pose.pose.position.z = base_to_tcp.getOrigin().z();
        tcp_pose.pose.orientation = tf2::toMsg(base_to_tcp.getRotation());
        return true;
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "Visual gripper marker TF unavailable: base_link -> %s (%s)",
            marker_frame.c_str(),
            ex.what());
        return false;
    }
}

bool VisualGraspApproach::moveBase(
    double distance_m,
    const ShouldContinue& should_continue) const
{
    if (std::abs(distance_m) < 1e-6)
        return true;

    const double speed = std::max(0.01, std::abs(base_final_approach_speed_mps_));
    const double sign = distance_m >= 0.0 ? 1.0 : -1.0;
    const double duration_s = std::abs(distance_m) / speed;
    const auto start = node_->now();
    rclcpp::Rate rate(30.0);

    while (rclcpp::ok() && (node_->now() - start).seconds() < duration_s)
    {
        if (!should_continue())
        {
            hardware_->MoveBase(0.0, 0.0, 0.0);
            return false;
        }
        hardware_->MoveBase(sign * speed, 0.0, 0.0);
        rate.sleep();
    }

    hardware_->MoveBase(0.0, 0.0, 0.0);
    return true;
}

bool VisualGraspApproach::buildReverseTrajectory(
    const std::string& arm_group,
    const std::vector<std::string>& joint_names,
    const std::vector<double>& source_waypoints,
    std::vector<double>& reversed_waypoints) const
{
    reversed_waypoints.clear();
    if (joint_names.empty() || source_waypoints.empty())
        return false;

    const std::size_t block_size = joint_names.size();
    if (source_waypoints.size() % block_size != 0)
        return false;

    const std::size_t blocks = source_waypoints.size() / block_size;
    if (blocks < 2)
        return false;

    reversed_waypoints.reserve(source_waypoints.size());
    for (std::size_t reverse_block = blocks; reverse_block > 0; --reverse_block)
    {
        const std::size_t source_block = reverse_block - 1;
        const auto first = source_waypoints.begin() + source_block * block_size;
        const auto last = first + block_size;
        reversed_waypoints.insert(reversed_waypoints.end(), first, last);
    }

    const std::string wrist_joint =
        (arm_group == "left_arm_group") ? "l_wrist_joint" : "r_wrist_joint";
    const auto wrist_it =
        std::find(joint_names.begin(), joint_names.end(), wrist_joint);

    if (wrist_it != joint_names.end())
    {
        const std::size_t wrist_index =
            static_cast<std::size_t>(std::distance(joint_names.begin(), wrist_it));
        RCLCPP_INFO(node_->get_logger(),
            "Built visual grasp reversed retract: blocks=%zu joints=%zu wrist=%s start=%.3f end=%.3f",
            blocks,
            block_size,
            wrist_joint.c_str(),
            reversed_waypoints[wrist_index],
            reversed_waypoints[(blocks - 1) * block_size + wrist_index]);
    }

    return true;
}

geometry_msgs::msg::PoseStamped VisualGraspApproach::applyOffset(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Vector3& offset)
{
    auto result = pose;
    result.pose.position.x += offset.x;
    result.pose.position.y += offset.y;
    result.pose.position.z += offset.z;
    return result;
}

double VisualGraspApproach::clamp(double value, double minimum, double maximum)
{
    return std::max(minimum, std::min(maximum, value));
}

bool VisualGraspApproach::prefersTopGrasp(const std::string& object_name)
{
    static const std::unordered_set<std::string> top_objects = {
        "plate", "bowl", "tray", "saucer", "box", "book"
    };
    return top_objects.count(object_name) > 0;
}
