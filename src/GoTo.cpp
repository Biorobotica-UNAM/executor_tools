#include "executor_tools/GoTo.hpp"

// =========================================================
// Constructor
// =========================================================
GoTo::GoTo(const rclcpp::Node::SharedPtr&     node,
		   const std::shared_ptr<Hardware>&   hardware,
           const std::shared_ptr<Navigation>& navigation)
: node_(node), hardware_(hardware), navigation_(navigation)
{}

// =========================================================
// run
// =========================================================
Result GoTo::run(const std::string&                 location_name,
                 const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, GoToState::READY, location_name);

	// ── MOVING_TORSO ─────────────────────────────────────
    publishFeedback(gh, GoToState::MOVING_TORSO);
    bool torso_moved = hardware_->MoveTorso(0.3);
    if (!torso_moved) {
        publishFeedback(gh, GoToState::FAILED, "move_torso_failed");
        return {false, "FAILED", "move_torso_failed"};
    }

	// ── MOVING_ARMS ─────────────────────────────────────
    publishFeedback(gh, GoToState::MOVING_ARMS);
	bool left_arm_moved = hardware_->MoveLeftArm({LEFT_ARM_NAVIGATING[0], LEFT_ARM_NAVIGATING[1], LEFT_ARM_NAVIGATING[2],
													LEFT_ARM_NAVIGATING[3], LEFT_ARM_NAVIGATING[4], LEFT_ARM_NAVIGATING[5]})
	if (!left_arm_moved) {
        publishFeedback(gh, GiveState::FAILED, "left_arm_navigation_failed");
        return {false, "FAILED", "left_arm_navigation_failed"};
    }
	bool right_arm_moved = hardware_->MoveRightArm({RIGHT_ARM_NAVIGATING[0], RIGHT_ARM_NAVIGATING[1], RIGHT_ARM_NAVIGATING[2],
													RIGHT_ARM_NAVIGATING[3], RIGHT_ARM_NAVIGATING[4], RIGHT_ARM_NAVIGATING[5]})
	if (!right_arm_moved) {
        publishFeedback(gh, GiveState::FAILED, "right_arm_navigation_failed");
        return {false, "FAILED", "right_arm_navigation_failed"};
    }

    // ── NAVIGATING ────────────────────────────────────────
    publishFeedback(gh, GoToState::NAVIGATING, location_name);

    auto nav_result = navigation_->GoToDetailed(location_name);

    // ── SUCCESS / FAILED ──────────────────────────────────
    if (nav_result.success) {
        publishFeedback(gh, GoToState::SUCCESS, location_name);
        return {true, "SUCCESS", "arrived_at: " + location_name + ": " + nav_result.message};
    } else {
        publishFeedback(gh, GoToState::FAILED, location_name + ": " + nav_result.message);
        return {false, "FAILED", "navigation_failed: " + location_name + ": " + nav_result.message};
    }
}

// =========================================================
// publishFeedback
// =========================================================
void GoTo::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                           GoToState                          state,
                           const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[go_to] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string GoTo::stateName(GoToState s)
{
    switch (s) {
        case GoToState::READY:      return "READY";
		case GoToState::MOVING_TORSO: return "MOVING_TORSO";
		case GoToState::MOVING_ARMS: return "MOVING_ARMS";
        case GoToState::NAVIGATING: return "NAVIGATING";
        case GoToState::SUCCESS:    return "SUCCESS";
        case GoToState::FAILED:     return "FAILED";
        default:                    return "UNKNOWN";
    }
}
