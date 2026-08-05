#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "hardware_tools/Hardware.hpp"
#include "manipulation_tools/Manipulation.hpp"

class VisualGraspApproach
{
public:
    enum class GraspMode
    {
        FRONT,
        TOP
    };

    struct Candidate
    {
        std::string label;
        GraspMode mode{GraspMode::FRONT};
        geometry_msgs::msg::Vector3 pregrasp_offset;
        geometry_msgs::msg::Vector3 grasp_offset;
    };

    struct PlanResult
    {
        bool success{false};
        std::string message;
        Candidate candidate;
        geometry_msgs::msg::PoseStamped object_pose;
        geometry_msgs::msg::PoseStamped pregrasp_pose;
        geometry_msgs::msg::PoseStamped grasp_pose;
        std::vector<std::string> joint_names;
        std::vector<double> waypoints;
        double total_base_advance_m{0.0};
    };

    using ShouldContinue = std::function<bool()>;

    VisualGraspApproach(
        const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<Manipulation>& manipulation,
        const std::shared_ptr<Hardware>& hardware);

    PlanResult planPregrasp(
        const std::string& object_name,
        const geometry_msgs::msg::PoseStamped& object_pose,
        const std::string& arm_group,
        const std::string& tip_link);

    bool executePregrasp(
        const std::string& arm_group,
        PlanResult& result);

    bool executeFinalApproach(
        const std::string& arm_group,
        const std::string& tip_link,
        PlanResult& result,
        const ShouldContinue& should_continue);

    bool retreatFinalApproach(
        PlanResult& result,
        const ShouldContinue& should_continue);

    bool buildReverseTrajectory(
        const std::string& arm_group,
        const std::vector<std::string>& joint_names,
        const std::vector<double>& source_waypoints,
        std::vector<double>& reversed_waypoints) const;

private:
    struct MotorStage
    {
        std::string label;
        std::vector<std::size_t> motors;
        std::size_t waypoints;
    };

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Manipulation> manipulation_;
    std::shared_ptr<Hardware> hardware_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool use_base_final_approach_{true};
    bool use_cartesian_final_approach_{false};
    bool use_visual_servo_{true};
    double base_final_approach_distance_m_{0.02};
    double base_final_approach_speed_mps_{0.05};
    int visual_max_attempts_{3};
    double visual_min_step_m_{0.005};
    double visual_max_step_m_{0.03};
    double visual_max_total_correction_m_{0.06};
    double front_desired_tcp_object_offset_m_{0.02};
    double front_tolerance_m_{0.01};
    double tf_lookup_timeout_sec_{0.25};
    std::string right_marker_frame_;
    std::string left_marker_frame_;
    std::vector<double> right_marker_to_tcp_xyz_;
    std::vector<double> left_marker_to_tcp_xyz_;

    template <typename T>
    T declareOrGet(const std::string& name, const T& default_value)
    {
        if (!node_->has_parameter(name))
            node_->declare_parameter<T>(name, default_value);
        return node_->get_parameter(name).get_value<T>();
    }

    std::vector<Candidate> candidatesForObject(const std::string& object_name) const;
    std::vector<MotorStage> approachMotorStages() const;
    std::size_t maxRemainingApproachBlocks() const;
    std::string motorJointName(const std::string& arm_group, std::size_t motor) const;

    bool orderApproachJointSequence(
        const std::string& arm_group,
        const std::vector<std::string>& joint_names,
        std::vector<double>& waypoints) const;

    bool executeVisualServoFront(
        const std::string& arm_group,
        PlanResult& result,
        const ShouldContinue& should_continue);

    bool lookupVisualTcp(
        const std::string& arm_group,
        geometry_msgs::msg::PoseStamped& tcp_pose) const;

    bool moveBase(
        double distance_m,
        const ShouldContinue& should_continue) const;

    static geometry_msgs::msg::PoseStamped applyOffset(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Vector3& offset);

    static double clamp(double value, double minimum, double maximum);
    static bool prefersTopGrasp(const std::string& object_name);
};
