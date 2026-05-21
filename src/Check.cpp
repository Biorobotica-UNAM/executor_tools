#include "executor_tools/Check.hpp"

#include <algorithm>
#include <cctype>

namespace
{
std::string normalizeCheckArgument(std::string value)
{
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool isDoorCheckArgument(const std::string& argument)
{
    const std::string normalized = normalizeCheckArgument(argument);
    return normalized == "door" ||
           normalized == "door_open" ||
           normalized == "open_door";
}

bool isContainerCheckArgument(const std::string& argument)
{
    const std::string normalized = normalizeCheckArgument(argument);
    return normalized == "container";
}
}  // namespace

// =========================================================
// Constructor
// =========================================================
Check::Check(const rclcpp::Node::SharedPtr&   node,
             const std::shared_ptr<Hardware>& hardware)
: node_(node), hardware_(hardware)
{}

// =========================================================
// run
// =========================================================
Result Check::run(const std::string&                 argument,
                  const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    // Check has explicit targets. Door is implemented; container is
    // recognized but not available yet; everything else is rejected early.
    if (isContainerCheckArgument(argument)) {
        publishFeedback(gh, CheckState::FAILED, "check_not_available: " + argument);
        return {false, "FAILED", "check_not_available: " + argument};
    }

    if (!isDoorCheckArgument(argument)) {
        publishFeedback(gh, CheckState::FAILED, "unsupported_check: " + argument);
        return {false, "FAILED", "unsupported_check: " + argument};
    }

    // Without the hardware facade this is an infrastructure failure, not a closed door.
    if (!hardware_) {
        return {false, "FAILED", "check_service_failed: hardware_unavailable"};
    }

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, CheckState::READY, argument);

    // ── CHECKING ──────────────────────────────────────────
    publishFeedback(gh, CheckState::CHECKING, argument);

    const auto door_state = hardware_->CheckDoorState();

    // ── SUCCESS / FAILED ──────────────────────────────────
    // SUCCESS is reserved for the only condition that lets the plan continue:
    // the door detector confirmed that the door is open.
    switch (door_state) {
        case Hardware::DoorCheckResult::OPEN:
            publishFeedback(gh, CheckState::SUCCESS, argument);
            return {true, "SUCCESS", "door_open: " + argument};

        case Hardware::DoorCheckResult::CLOSED:
            // A closed door is a valid sensor reading; the supervisor may ask a
            // human to open it and retry this same check.
            publishFeedback(gh, CheckState::FAILED, "door_closed");
            return {false, "FAILED", "door_closed: " + argument};

        case Hardware::DoorCheckResult::SERVICE_FAILED:
            // Service failures should not start the door-wait loop.
            publishFeedback(gh, CheckState::FAILED, "check_service_failed");
            return {false, "FAILED", "check_service_failed: " + argument};
    }

    // Defensive fallback for future enum values.
    publishFeedback(gh, CheckState::FAILED, "unknown_door_state");
    return {false, "FAILED", "check_service_failed: unknown_door_state"};
}

// =========================================================
// publishFeedback
// =========================================================
void Check::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                            CheckState                         state,
                            const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[check] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string Check::stateName(CheckState s)
{
    switch (s) {
        case CheckState::READY:    return "READY";
        case CheckState::CHECKING: return "CHECKING";
        case CheckState::SUCCESS:  return "SUCCESS";
        case CheckState::FAILED:   return "FAILED";
        default:                   return "UNKNOWN";
    }
}
