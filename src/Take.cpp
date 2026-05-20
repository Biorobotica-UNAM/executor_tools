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
    const std::shared_ptr<Manipulation>&  manipulation)
: node_(node),
  manipulation_(manipulation)
{}

// ─────────────────────────────────────────────────────────────
//  run
// ─────────────────────────────────────────────────────────────
Result Take::run(
    const std::string&                     arm,
    const geometry_msgs::msg::PoseStamped& target_pose,
    const std::shared_ptr<GoalHandle>&     gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_inactive"};

    if (arm != "right_arm_group" && arm != "left_arm_group")
    {
        RCLCPP_ERROR(node_->get_logger(),
            "[take] Brazo inválido: '%s'. Usa 'right_arm_group' o 'left_arm_group'",
            arm.c_str());
        publishFeedback(gh, TakeState::FAILED, "invalid_arm");
        return {false, "FAILED", "invalid_arm"};
    }

    publishFeedback(gh, TakeState::READY, arm);

    // ── PLAN_REACH ────────────────────────────────────────────
    publishFeedback(gh, TakeState::PLAN_REACH);

    bool ok = attempt("PLAN_REACH", gh, [&](std::string& msg) {
        return manipulation_->PlanTrajectory(
            target_pose, arm, joint_names_, waypoints_, msg);
    });

    if (!ok)
        return {false, "FAILED", "plan_reach_failed"};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── EXECUTE_REACH ─────────────────────────────────────────
    publishFeedback(gh, TakeState::EXECUTE_REACH);

    ok = attempt("EXECUTE_REACH", gh, [&](std::string& msg) {
        return manipulation_->FollowTrajectory(
            arm, joint_names_, waypoints_, msg);
    });

    if (!ok)
        return {false, "FAILED", "execute_reach_failed"};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── CLOSE_GRIPPER ─────────────────────────────────────────
    publishFeedback(gh, TakeState::CLOSE_GRIPPER);

    ok = attempt("CLOSE_GRIPPER", gh, [&](std::string& msg) {
        // TODO: reemplazar con llamada real al gripper
        RCLCPP_INFO(node_->get_logger(), "[take] Cerrando gripper (fake)");
        std::this_thread::sleep_for(1s);
        msg = "";
        return true;
    });

    if (!ok)
        return {false, "FAILED", "close_gripper_failed"};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── PLAN_RETRACT ──────────────────────────────────────────
    publishFeedback(gh, TakeState::PLAN_RETRACT);

    const auto home = retractPose(arm);

    ok = attempt("PLAN_RETRACT", gh, [&](std::string& msg) {
        return manipulation_->PlanTrajectory(
            home, arm, joint_names_, waypoints_, msg);
    });

    if (!ok)
        return {false, "FAILED", "plan_retract_failed"};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── EXECUTE_RETRACT ───────────────────────────────────────
    publishFeedback(gh, TakeState::EXECUTE_RETRACT);

    ok = attempt("EXECUTE_RETRACT", gh, [&](std::string& msg) {
        return manipulation_->FollowTrajectory(
            arm, joint_names_, waypoints_, msg);
    });

    if (!ok)
        return {false, "FAILED", "execute_retract_failed"};

    // ── SUCCESS ───────────────────────────────────────────────
    publishFeedback(gh, TakeState::SUCCESS, arm);
    return {true, "SUCCESS", arm + "_take_completed"};
}

// ─────────────────────────────────────────────────────────────
//  attempt
// ─────────────────────────────────────────────────────────────
bool Take::attempt(
    const std::string&                 step,
    const std::shared_ptr<GoalHandle>& gh,
    std::function<bool(std::string&)>  fn)
{
    for (int i = 1; i <= MAX_ATTEMPTS; ++i)
    {
        if (!gh->is_active())
            return false;

        RCLCPP_INFO(node_->get_logger(),
            "[take] %s — intento %d/%d", step.c_str(), i, MAX_ATTEMPTS);

        std::string msg;
        if (fn(msg))
        {
            RCLCPP_INFO(node_->get_logger(),
                "[take] %s completado en intento %d/%d",
                step.c_str(), i, MAX_ATTEMPTS);
            return true;
        }

        RCLCPP_WARN(node_->get_logger(),
            "[take] %s falló (intento %d/%d): %s",
            step.c_str(), i, MAX_ATTEMPTS, msg.c_str());
    }

    RCLCPP_ERROR(node_->get_logger(),
        "[take] %s agotó %d intentos — abortando",
        step.c_str(), MAX_ATTEMPTS);
    return false;
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
        case TakeState::READY:           return "READY";
        case TakeState::PLAN_REACH:      return "PLAN_REACH";
        case TakeState::EXECUTE_REACH:   return "EXECUTE_REACH";
        case TakeState::CLOSE_GRIPPER:   return "CLOSE_GRIPPER";
        case TakeState::PLAN_RETRACT:    return "PLAN_RETRACT";
        case TakeState::EXECUTE_RETRACT: return "EXECUTE_RETRACT";
        case TakeState::SUCCESS:         return "SUCCESS";
        case TakeState::FAILED:          return "FAILED";
        default:                         return "UNKNOWN";
    }
}