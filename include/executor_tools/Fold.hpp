#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"
#include "vision_tools/Vision.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <array>
#include <chrono>
#include <memory>
#include <string>

// ─────────────────────────────────────────────────────────────
//  FSM states
// ─────────────────────────────────────────────────────────────
enum class FoldState {
    READY,
    LOOKING_AT_CLOTH,       // head + torso pre-positioning
    MOVING_TORSO,           // raise torso for table reach
    DETECTING_CLOTH,        // vision: GetClothKeypoints()
    APPROACHING_SLEEVES,    // both arms move to sleeve positions
    GRIPPING_SLEEVES,       // close grippers on sleeves
    LIFTING_SLEEVES,        // raise both arms
    FOLDING_LEFT_SLEEVE,    // left arm sweeps inward → open gripper
    FOLDING_RIGHT_SLEEVE,   // right arm sweeps inward → open gripper
    APPROACHING_BOTTOM,     // both arms move to bottom corners
    GRIPPING_BOTTOM,        // close grippers on bottom edge
    LIFTING_AND_ADVANCING,  // lift arms + drive base forward
    RELEASING_BOTTOM,       // open grippers
    RESETTING,              // arms to rest, torso and head to neutral
    SUCCESS,
    FAILED
};

// ─────────────────────────────────────────────────────────────
//  Fold
//  Folds a flat t-shirt lying on a table with its bottom
//  hanging off the near edge:
//    Phase 1 — Sleeves: grip at X extremes, ~75 % down Y bbox,
//              lift, sweep each one inward, release.
//    Phase 2 — Bottom: grip bottom corners, lift, drive base
//              forward to drop bottom half on top half, release.
// ─────────────────────────────────────────────────────────────
class Fold
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Fold(const rclcpp::Node::SharedPtr&   node,
         const std::shared_ptr<Hardware>& hardware,
         const std::shared_ptr<Vision>&   vision);

    Result run(const std::string&                 argument,
               const std::shared_ptr<GoalHandle>& gh);

private:
    // ── Sub-steps (each returns early on failure) ─────────────
    Result stepDetectCloth  (const std::shared_ptr<GoalHandle>& gh);
    Result stepTakeSleeves  (const std::shared_ptr<GoalHandle>& gh);
    Result stepFoldSleeves  (const std::shared_ptr<GoalHandle>& gh);
    Result stepTakeBottom   (const std::shared_ptr<GoalHandle>& gh);
    Result stepLiftAndAdvance(const std::shared_ptr<GoalHandle>& gh);
    Result stepReset        (const std::shared_ptr<GoalHandle>& gh);

    // ── Time-based linear base drive (mirrors Point's rotation) ─
    bool driveForward(double distance_m,
                      const std::shared_ptr<GoalHandle>& gh);

    // ── Feedback / logging ────────────────────────────────────
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         FoldState                           state,
                         const std::string&                  detail = "");
    static std::string stateName(FoldState s);

    // ═══════════════════════════════════════════════════════════
    //  Kinematic constants — ALL TODO: tune for Justina.
    //  Sleeve poses target the shirt with arms at ~75 % Y depth
    //  of the bounding box; bottom poses target the hanging edge.
    //  Replace with empirical or IK-computed joint angles.
    // ═══════════════════════════════════════════════════════════

    // Phase 1 – Sleeves
    static constexpr std::array<double,6> LEFT_ARM_SLEEVE_APPROACH  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static constexpr std::array<double,6> RIGHT_ARM_SLEEVE_APPROACH = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static constexpr std::array<double,6> LEFT_ARM_SLEEVE_LIFTED    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static constexpr std::array<double,6> RIGHT_ARM_SLEEVE_LIFTED   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static constexpr std::array<double,6> LEFT_ARM_SLEEVE_FOLDED    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // inward sweep
    static constexpr std::array<double,6> RIGHT_ARM_SLEEVE_FOLDED   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // inward sweep

    // Phase 2 – Bottom
    static constexpr std::array<double,6> LEFT_ARM_BOTTOM_APPROACH  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static constexpr std::array<double,6> RIGHT_ARM_BOTTOM_APPROACH = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static constexpr std::array<double,6> LEFT_ARM_BOTTOM_LIFTED    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static constexpr std::array<double,6> RIGHT_ARM_BOTTOM_LIFTED   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Neutral
    static constexpr std::array<double,6> ARM_RESTING               = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // ── Motion parameters ─────────────────────────────────────
    static constexpr double TORSO_FOLD_HEIGHT = 0.15;  // m   TODO: tune
    static constexpr double HEAD_TILT_TABLE   = -0.55; // rad TODO: tune for table height
    static constexpr double LINEAR_SPEED      = 0.08;  // m/s (conservative)
    static constexpr double FORWARD_DISTANCE  = 0.15;  // m   TODO: tune

    // ── Members ───────────────────────────────────────────────
    rclcpp::Node::SharedPtr   node_;
    std::shared_ptr<Hardware> hardware_;
    std::shared_ptr<Vision>   vision_;

    // Cloth keypoints cached after detection (world frame)
    geometry_msgs::msg::PointStamped cloth_left_sleeve_;
    geometry_msgs::msg::PointStamped cloth_right_sleeve_;
    geometry_msgs::msg::PointStamped cloth_bottom_left_;
    geometry_msgs::msg::PointStamped cloth_bottom_right_;
    geometry_msgs::msg::PointStamped cloth_center_;
};
