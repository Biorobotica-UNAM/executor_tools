#include "executor_tools/Fold.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
Fold::Fold(const rclcpp::Node::SharedPtr&   node,
           const std::shared_ptr<Hardware>& hardware,
           const std::shared_ptr<HRI>& hri,
           const std::shared_ptr<Vision>&   vision)
: node_(node), hardware_(hardware), hri_(hri), vision_(vision)
{}

// ─────────────────────────────────────────────────────────────
//  run
// ─────────────────────────────────────────────────────────────
Result Fold::run(const std::string&                 argument,
                 const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    publishFeedback(gh, FoldState::READY, argument);

    // Phase 0: position head/torso + wait for human
    hardware_->MoveHead({0.0, HEAD_TILT_TABLE});
    //hardware_->MoveTorso(TORSO_FOLD_HEIGHT);
    hri_->Say("Please lay the shirt flat on the table.");
    std::this_thread::sleep_for(10s);

    // Phase 1: left sleeve
    auto r = redetect(gh);
    if (!r.success) return r;
    r = stepTakeLeftSleeve(gh);
    if (!r.success) return r;
    r = stepFoldLeftSleeve(gh);
    if (!r.success) return r;

    // Phase 2: right sleeve (cloth changed shape)
    r = redetect(gh);
    if (!r.success) return r;
    r = stepTakeRightSleeve(gh);
    if (!r.success) return r;
    r = stepFoldRightSleeve(gh);
    if (!r.success) return r;

    // Phase 3: bottom fold (cloth changed shape again)
    r = redetect(gh);
    if (!r.success) return r;
    r = stepTakeBottom(gh);
    if (!r.success) return r;
    r = stepLiftAndAdvance(gh);
    if (!r.success) return r;

    r = stepReset(gh);
    if (!r.success) return r;

    publishFeedback(gh, FoldState::SUCCESS, "cloth_folded");
    return {true, "SUCCESS", "cloth_folded_successfully"};
}

// ─────────────────────────────────────────────────────────────
//  redetect
//  Fresh YOLO detection before every "take" step.
//  Retries up to 3 times, then caches keypoints and centres head.
// ─────────────────────────────────────────────────────────────
Result Fold::redetect(const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh->is_active()) return {false, "FAILED", "goal_cancelled"};
    publishFeedback(gh, FoldState::DETECTING_CLOTH);

    Vision::ClothKeypoints kp;
    bool detected = false;

    for (int attempt = 1; attempt <= 3 && !detected; ++attempt) {
        detected = vision_->GetClothKeypoints(kp, 3s);
        if (!detected) {
            RCLCPP_WARN(node_->get_logger(),
                        "[Fold] redetect attempt %d/3 failed", attempt);
            std::this_thread::sleep_for(300ms);
        }
    }

    if (!detected) {
        publishFeedback(gh, FoldState::FAILED, "cloth_not_detected");
        return {false, "FAILED", "cloth_not_detected"};
    }

    cloth_left_sleeve_  = kp.left_sleeve;
    cloth_right_sleeve_ = kp.right_sleeve;
    cloth_bottom_left_  = kp.bottom_left;
    cloth_bottom_right_ = kp.bottom_right;
    cloth_center_       = kp.center;

    hardware_->LookAt(cloth_center_);
    std::this_thread::sleep_for(400ms);

    return {true, "DETECTING_CLOTH", "cloth_detected"};
}

// ─────────────────────────────────────────────────────────────
//  stepTakeLeftSleeve / stepFoldLeftSleeve
// ─────────────────────────────────────────────────────────────
Result Fold::stepTakeLeftSleeve(const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh->is_active()) return {false, "FAILED", "goal_cancelled"};
    publishFeedback(gh, FoldState::APPROACHING_SLEEVES);
    hardware_->MoveLeftGripper("open");
    bool ok = hardware_->MoveLeftArm({
        LEFT_ARM_SLEEVE_APPROACH[0], LEFT_ARM_SLEEVE_APPROACH[1],
        LEFT_ARM_SLEEVE_APPROACH[2], LEFT_ARM_SLEEVE_APPROACH[3],
        LEFT_ARM_SLEEVE_APPROACH[4], LEFT_ARM_SLEEVE_APPROACH[5]});
    if (!ok) return {false, "FAILED", "left_sleeve_approach_failed"};
    publishFeedback(gh, FoldState::GRIPPING_SLEEVES);
    hardware_->MoveLeftGripper("close");
    std::this_thread::sleep_for(300ms);
    return {true, "GRIPPING_SLEEVES", "left_sleeve_gripped"};
}

Result Fold::stepFoldLeftSleeve(const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh->is_active()) return {false, "FAILED", "goal_cancelled"};
    publishFeedback(gh, FoldState::LIFTING_SLEEVES);
    hardware_->MoveLeftArm({
        LEFT_ARM_SLEEVE_LIFTED[0], LEFT_ARM_SLEEVE_LIFTED[1],
        LEFT_ARM_SLEEVE_LIFTED[2], LEFT_ARM_SLEEVE_LIFTED[3],
        LEFT_ARM_SLEEVE_LIFTED[4], LEFT_ARM_SLEEVE_LIFTED[5]});
    std::this_thread::sleep_for(500ms);
    publishFeedback(gh, FoldState::FOLDING_LEFT_SLEEVE);
    bool ok = hardware_->MoveLeftArm({
        LEFT_ARM_SLEEVE_FOLDED[0], LEFT_ARM_SLEEVE_FOLDED[1],
        LEFT_ARM_SLEEVE_FOLDED[2], LEFT_ARM_SLEEVE_FOLDED[3],
        LEFT_ARM_SLEEVE_FOLDED[4], LEFT_ARM_SLEEVE_FOLDED[5]});
    if (!ok) return {false, "FAILED", "left_sleeve_fold_failed"};
    hardware_->MoveLeftGripper("open");
    std::this_thread::sleep_for(400ms);
    return {true, "FOLDING_LEFT_SLEEVE", "left_sleeve_folded"};
}

// ─────────────────────────────────────────────────────────────
//  stepTakeRightSleeve / stepFoldRightSleeve
// ─────────────────────────────────────────────────────────────
Result Fold::stepTakeRightSleeve(const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh->is_active()) return {false, "FAILED", "goal_cancelled"};
    publishFeedback(gh, FoldState::APPROACHING_SLEEVES);
    hardware_->MoveRightGripper("open");
    bool ok = hardware_->MoveRightArm({
        RIGHT_ARM_SLEEVE_APPROACH[0], RIGHT_ARM_SLEEVE_APPROACH[1],
        RIGHT_ARM_SLEEVE_APPROACH[2], RIGHT_ARM_SLEEVE_APPROACH[3],
        RIGHT_ARM_SLEEVE_APPROACH[4], RIGHT_ARM_SLEEVE_APPROACH[5]});
    if (!ok) return {false, "FAILED", "right_sleeve_approach_failed"};
    publishFeedback(gh, FoldState::GRIPPING_SLEEVES);
    hardware_->MoveRightGripper("close");
    std::this_thread::sleep_for(300ms);
    return {true, "GRIPPING_SLEEVES", "right_sleeve_gripped"};
}

Result Fold::stepFoldRightSleeve(const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh->is_active()) return {false, "FAILED", "goal_cancelled"};
    publishFeedback(gh, FoldState::LIFTING_SLEEVES);
    hardware_->MoveRightArm({
        RIGHT_ARM_SLEEVE_LIFTED[0], RIGHT_ARM_SLEEVE_LIFTED[1],
        RIGHT_ARM_SLEEVE_LIFTED[2], RIGHT_ARM_SLEEVE_LIFTED[3],
        RIGHT_ARM_SLEEVE_LIFTED[4], RIGHT_ARM_SLEEVE_LIFTED[5]});
    std::this_thread::sleep_for(500ms);
    publishFeedback(gh, FoldState::FOLDING_RIGHT_SLEEVE);
    bool ok = hardware_->MoveRightArm({
        RIGHT_ARM_SLEEVE_FOLDED[0], RIGHT_ARM_SLEEVE_FOLDED[1],
        RIGHT_ARM_SLEEVE_FOLDED[2], RIGHT_ARM_SLEEVE_FOLDED[3],
        RIGHT_ARM_SLEEVE_FOLDED[4], RIGHT_ARM_SLEEVE_FOLDED[5]});
    if (!ok) return {false, "FAILED", "right_sleeve_fold_failed"};
    hardware_->MoveRightGripper("open");
    std::this_thread::sleep_for(400ms);
    return {true, "FOLDING_RIGHT_SLEEVE", "right_sleeve_folded"};
}

// ─────────────────────────────────────────────────────────────
//  stepTakeBottom
//  Open grippers → move both arms to bottom corners →
//  close grippers on the hanging edge.
// ─────────────────────────────────────────────────────────────
Result Fold::stepTakeBottom(const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh->is_active()) return {false, "FAILED", "goal_cancelled"};
    publishFeedback(gh, FoldState::APPROACHING_BOTTOM);

    // ── Pre-open grippers ─────────────────────────────────────
    hardware_->MoveLeftGripper("open");
    hardware_->MoveRightGripper("open");
    std::this_thread::sleep_for(300ms);

    // ── Move both arms to the bottom corners ──────────────────
    // Cartesian reference (world frame):
    //   left  arm target = cloth_bottom_left_   (X_min, Y_max — hanging edge)
    //   right arm target = cloth_bottom_right_  (X_max, Y_max — hanging edge)
    // TODO: replace joint arrays with IK-derived or empirically tuned values.
    bool left_ok  = hardware_->MoveLeftArm({
        LEFT_ARM_BOTTOM_APPROACH[0],  LEFT_ARM_BOTTOM_APPROACH[1],
        LEFT_ARM_BOTTOM_APPROACH[2],  LEFT_ARM_BOTTOM_APPROACH[3],
        LEFT_ARM_BOTTOM_APPROACH[4],  LEFT_ARM_BOTTOM_APPROACH[5]});

    bool right_ok = hardware_->MoveRightArm({
        RIGHT_ARM_BOTTOM_APPROACH[0], RIGHT_ARM_BOTTOM_APPROACH[1],
        RIGHT_ARM_BOTTOM_APPROACH[2], RIGHT_ARM_BOTTOM_APPROACH[3],
        RIGHT_ARM_BOTTOM_APPROACH[4], RIGHT_ARM_BOTTOM_APPROACH[5]});

    if (!left_ok || !right_ok) {
        publishFeedback(gh, FoldState::FAILED, "bottom_approach_arm_failed");
        return {false, "FAILED", "bottom_approach_arm_failed"};
    }

    // ── Close grippers on the bottom edge ────────────────────
    publishFeedback(gh, FoldState::GRIPPING_BOTTOM);
    std::this_thread::sleep_for(300ms);

    bool lg_ok = hardware_->MoveLeftGripper("close");
    bool rg_ok = hardware_->MoveRightGripper("close");

    if (!lg_ok || !rg_ok) {
        publishFeedback(gh, FoldState::FAILED, "bottom_grip_failed");
        return {false, "FAILED", "bottom_grip_failed"};
    }

    std::this_thread::sleep_for(300ms);
    return {true, "GRIPPING_BOTTOM", "bottom_gripped"};
}

// ─────────────────────────────────────────────────────────────
//  stepLiftAndAdvance
//  Lift arms holding the bottom edge → drive base forward
//  (mirrors Point's open-loop base drive, but linear) →
//  deposit the bottom half over the top half → open grippers.
// ─────────────────────────────────────────────────────────────
Result Fold::stepLiftAndAdvance(const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh->is_active()) return {false, "FAILED", "goal_cancelled"};
    publishFeedback(gh, FoldState::LIFTING_AND_ADVANCING);

    // ── Lift both arms while holding the shirt bottom ─────────
    // TODO: LIFTED poses should keep the bottom edge above the table surface.
    bool left_up  = hardware_->MoveLeftArm({
        LEFT_ARM_BOTTOM_LIFTED[0],  LEFT_ARM_BOTTOM_LIFTED[1],
        LEFT_ARM_BOTTOM_LIFTED[2],  LEFT_ARM_BOTTOM_LIFTED[3],
        LEFT_ARM_BOTTOM_LIFTED[4],  LEFT_ARM_BOTTOM_LIFTED[5]});

    bool right_up = hardware_->MoveRightArm({
        RIGHT_ARM_BOTTOM_LIFTED[0], RIGHT_ARM_BOTTOM_LIFTED[1],
        RIGHT_ARM_BOTTOM_LIFTED[2], RIGHT_ARM_BOTTOM_LIFTED[3],
        RIGHT_ARM_BOTTOM_LIFTED[4], RIGHT_ARM_BOTTOM_LIFTED[5]});

    if (!left_up || !right_up) {
        publishFeedback(gh, FoldState::FAILED, "bottom_lift_arm_failed");
        return {false, "FAILED", "bottom_lift_arm_failed"};
    }
    std::this_thread::sleep_for(300ms);

    // ── Drive base forward to carry the lifted bottom over the shirt ──
    // Open-loop time-based drive: same pattern as Point::enable()
    // rotating the base, but here applied linearly.
    // FORWARD_DISTANCE controls how far we advance; tune so the bottom
    // half drops aligned with the collar end of the shirt.
    if (!driveForward(FORWARD_DISTANCE, gh)) {
        return {false, "FAILED", "base_advance_cancelled"};
    }

    // ── Release — deposit bottom half on top half ─────────────
    publishFeedback(gh, FoldState::RELEASING_BOTTOM);
    hardware_->MoveLeftGripper("open");
    hardware_->MoveRightGripper("open");
    std::this_thread::sleep_for(400ms);

    return {true, "RELEASING_BOTTOM", "bottom_released"};
}

// ─────────────────────────────────────────────────────────────
//  stepReset
//  Return arms to neutral, drive base back slightly if needed,
//  restore torso and head to default positions.
// ─────────────────────────────────────────────────────────────
Result Fold::stepReset(const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh->is_active()) return {false, "FAILED", "goal_cancelled"};
    publishFeedback(gh, FoldState::RESETTING);

    hardware_->MoveBase(0.0, 0.0, 0.0);

    hardware_->MoveLeftArm({ARM_RESTING[0],  ARM_RESTING[1],  ARM_RESTING[2],
                             ARM_RESTING[3],  ARM_RESTING[4],  ARM_RESTING[5]});
    hardware_->MoveRightArm({ARM_RESTING[0], ARM_RESTING[1], ARM_RESTING[2],
                              ARM_RESTING[3], ARM_RESTING[4], ARM_RESTING[5]});

    hardware_->MoveTorso(0.0);        // TODO: define Justina's neutral torso height
    hardware_->MoveHead({0.0, 0.0});  // forward-looking, level

    std::this_thread::sleep_for(500ms);
    return {true, "RESETTING", "robot_reset"};
}

// ─────────────────────────────────────────────────────────────
//  driveForward
//  Time-based open-loop linear advance.
//  Mirrors the rotation loop in Point::enable() but applies
//  a forward (X) velocity instead of angular (Z).
//  Stops immediately if the goal is cancelled.
// ─────────────────────────────────────────────────────────────
bool Fold::driveForward(double distance_m,
                        const std::shared_ptr<GoalHandle>& gh)
{
    const double duration_s = distance_m / LINEAR_SPEED;
    rclcpp::Rate rate(30.0);
    const auto   start = node_->now();

    while (rclcpp::ok() &&
           (node_->now() - start).seconds() < duration_s)
    {
        if (!gh || !gh->is_active()) {
            hardware_->MoveBase(0.0, 0.0, 0.0);
            return false;
        }
        hardware_->MoveBase(LINEAR_SPEED, 0.0, 0.0);
        rate.sleep();
    }

    hardware_->MoveBase(0.0, 0.0, 0.0);
    return true;
}

// ─────────────────────────────────────────────────────────────
//  publishFeedback
// ─────────────────────────────────────────────────────────────
void Fold::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                            FoldState                           state,
                            const std::string&                  detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[fold] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// ─────────────────────────────────────────────────────────────
//  stateName
// ─────────────────────────────────────────────────────────────
std::string Fold::stateName(FoldState s)
{
    switch (s) {
        case FoldState::READY:                 return "READY";
        case FoldState::LOOKING_AT_CLOTH:      return "LOOKING_AT_CLOTH";
        case FoldState::MOVING_TORSO:          return "MOVING_TORSO";
        case FoldState::DETECTING_CLOTH:       return "DETECTING_CLOTH";
        case FoldState::APPROACHING_SLEEVES:   return "APPROACHING_SLEEVES";
        case FoldState::GRIPPING_SLEEVES:      return "GRIPPING_SLEEVES";
        case FoldState::LIFTING_SLEEVES:       return "LIFTING_SLEEVES";
        case FoldState::FOLDING_LEFT_SLEEVE:   return "FOLDING_LEFT_SLEEVE";
        case FoldState::FOLDING_RIGHT_SLEEVE:  return "FOLDING_RIGHT_SLEEVE";
        case FoldState::APPROACHING_BOTTOM:    return "APPROACHING_BOTTOM";
        case FoldState::GRIPPING_BOTTOM:       return "GRIPPING_BOTTOM";
        case FoldState::LIFTING_AND_ADVANCING: return "LIFTING_AND_ADVANCING";
        case FoldState::RELEASING_BOTTOM:      return "RELEASING_BOTTOM";
        case FoldState::RESETTING:             return "RESETTING";
        case FoldState::SUCCESS:               return "SUCCESS";
        case FoldState::FAILED:                return "FAILED";
        default:                               return "UNKNOWN";
    }
}
