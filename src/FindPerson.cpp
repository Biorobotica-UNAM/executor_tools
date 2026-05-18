#include "executor_tools/FindPerson.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================

FindPerson::FindPerson(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<Hardware>& hardware,
    const std::shared_ptr<Vision>& vision)
: node_(node),
  hardware_(hardware),
  vision_(vision)
{}

// =========================================================
// run
// =========================================================

Result FindPerson::run(
    const std::string& argument,
    const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    publishFeedback(
        gh,
        FindPersonState::READY,
        argument);

    const bool anybody =
        (argument == "anybody");

    // =====================================================
    // SCAN CONFIG
    // =====================================================

    std::vector<double> scan_values;

    if (anybody)
    {
        // vertical scan
        scan_values =
        {
             0.45,
             0.25,
             0.0,
            -0.2
        };
    }
    else
    {
        // horizontal scan
        scan_values =
        {
            -1.2,
            -0.8,
            -0.4,
             0.0,
             0.4,
             0.8,
             1.2
        };
    }

    // =====================================================
    // SCANNING LOOP
    // =====================================================

    for (double value : scan_values)
    {
        if (!gh->is_active())
            return {
                false,
                "FAILED",
                "goal_cancelled"
            };

        publishFeedback(
            gh,
            FindPersonState::SCANNING);

        double pan  = 0.0;
        double tilt = 0.0;

        if (anybody)
        {
            tilt = value;
        }
        else
        {
            pan = value;
        }

        hardware_->MoveHead({pan, tilt});

        std::this_thread::sleep_for(1200ms);

        publishFeedback(
            gh,
            FindPersonState::DETECTING);

        bool found =
            anybody
            ? vision_->DetectPerson("")
            : vision_->IdentifyPerson(argument);

        if (found)
        {
            hardware_->MoveHead({0.0, 0.0});

            publishFeedback(
                gh,
                FindPersonState::SUCCESS,
                argument);

            return {
                true,
                "SUCCESS",
                "person_found: " + argument
            };
        }
    }

    // =====================================================
    // FAILED
    // =====================================================

    hardware_->MoveHead({0.0, 0.0});

    publishFeedback(
        gh,
        FindPersonState::FAILED,
        argument);

    return {
        false,
        "FAILED",
        "person_not_found: " + argument
    };
}

// =========================================================
// publishFeedback
// =========================================================

void FindPerson::publishFeedback(
    const std::shared_ptr<GoalHandle>& gh,
    FindPersonState state,
    const std::string& detail)
{
    if (!gh->is_active())
        return;

    std::string msg =
        "[find_person] " + stateName(state);

    if (!detail.empty())
        msg += ": " + detail;

    auto fb =
        std::make_shared<ExecuteTask::Feedback>();

    fb->feedback.message = msg;

    gh->publish_feedback(fb);

    RCLCPP_INFO(
        node_->get_logger(),
        "%s",
        msg.c_str());
}

// =========================================================
// stateName
// =========================================================

std::string FindPerson::stateName(
    FindPersonState s)
{
    switch (s)
    {
        case FindPersonState::READY:	return "READY";
        case FindPersonState::SCANNING:	return "SCANNING";
        case FindPersonState::DETECTING:return "DETECTING";
        case FindPersonState::SUCCESS:	return "SUCCESS";
        case FindPersonState::FAILED:	return "FAILED";
        default:						return "UNKNOWN";
    }
}
