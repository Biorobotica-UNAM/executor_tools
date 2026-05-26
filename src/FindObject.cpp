#include "executor_tools/FindObject.hpp"
#include "executor_tools/Result.hpp"

#include <chrono>
#include <thread>
#include <string>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────

FindObject::FindObject(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<Hardware>& hardware,
    const std::shared_ptr<Vision>& vision,
    const std::shared_ptr<Knowledge>& knowledge)
: node_(node),
  hardware_(hardware),
  vision_(vision),
  knowledge_(knowledge)
{}

// ─────────────────────────────────────────────────────────────
//  run
// ─────────────────────────────────────────────────────────────

Result FindObject::run(
    const std::string& argument,
    const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_inactive"};

    // ── READY ──────────────────────────────────────────────

    publishFeedback(
        gh,
        FindObjectState::READY,
        argument);

    std::string query_name;
    std::string object_index;

    if (!parseArgument(
            argument,
            query_name,
            object_index))
    {
        publishFeedback(
            gh,
            FindObjectState::FAILED,
            "invalid_argument");

        return {
            false,
            "FAILED",
            "invalid_argument"
        };
    }

    for (const auto& [pan, tilt] : scan_poses_)
    {
        if (!gh->is_active())
        {
            return {
                false,
                "FAILED",
                "goal_cancelled"
            };
        }

        // ── MOVING_HEAD ────────────────────────────────────

        publishFeedback(
            gh,
            FindObjectState::MOVING_HEAD);

        hardware_->MoveHead({pan, tilt});

        std::this_thread::sleep_for(1500ms);

        // ── STABILIZING ────────────────────────────────────

        publishFeedback(
            gh,
            FindObjectState::STABILIZING);

        std::this_thread::sleep_for(500ms);

        // ── DETECTING ──────────────────────────────────────

        publishFeedback(
            gh,
            FindObjectState::DETECTING);

        // ── EMPTY CHAIR ────────────────────────────────────

        if (query_name == "empty_chair")
        {
            geometry_msgs::msg::PointStamped seat_point;

            if (detectEmptyChair(seat_point))
            {
                publishFeedback(
                    gh,
                    FindObjectState::LOOKING_AT_TARGET);

                hardware_->LookAt(seat_point);

                publishFeedback(
                    gh,
                    FindObjectState::SUCCESS,
                    "empty_chair");

                return {
                    true,
                    "SUCCESS",
                    "empty_chair_found"
                };
            }
        }

        // ── OWL OBJECTS ────────────────────────────────────

        else
        {
            geometry_msgs::msg::PoseStamped object_pose;

            std::string detected_name;

            if (vision_->GetOwlPose(
                    query_name,
                    object_pose,
                    detected_name))
            {
                // ── LOOKING_AT_TARGET ─────────────────────

                publishFeedback(
                    gh,
                    FindObjectState::LOOKING_AT_TARGET,
                    detected_name);

                hardware_->LookAt(object_pose);

                // ── UPDATING ─────────────────────────────

                publishFeedback(
                    gh,
                    FindObjectState::UPDATING,
                    detected_name);

                const auto payload =
					buildManipulationPayload(
						query_name,
						object_index,
						detected_name,
						object_pose);

                const bool stored =
                    knowledge_->update_world_state(payload);

                if (!stored)
                {
                    publishFeedback(
                        gh,
                        FindObjectState::FAILED,
                        "update_world_state_failed");

                    return {
                        false,
                        "FAILED",
                        "update_world_state_failed"
                    };
                }

                // ── SUCCESS ──────────────────────────────

                publishFeedback(
                    gh,
                    FindObjectState::SUCCESS,
                    detected_name);

                return {
                    true,
                    "SUCCESS",
                    detected_name + "_found"
                };
            }
        }
    }

    publishFeedback(
        gh,
        FindObjectState::FAILED,
        query_name);

    return {
        false,
        "FAILED",
        query_name + "_not_found"
    };
}

// ─────────────────────────────────────────────────────────────
//  detectEmptyChair
// ─────────────────────────────────────────────────────────────

bool FindObject::detectEmptyChair(
    geometry_msgs::msg::PointStamped& point)
{
    auto result =
        vision_->FindSeat();

    if (!result.first)
        return false;

    point = result.second;

    return true;
}

// ─────────────────────────────────────────────────────────────
//  parseArgument
// ─────────────────────────────────────────────────────────────

bool FindObject::parseArgument(
    const std::string& argument,
    std::string& out_name,
    std::string& out_index) const
{
    const std::size_t sep =
        argument.rfind('_');

    if (sep == std::string::npos ||
        sep == 0 ||
        sep + 1 == argument.size())
    {
        return false;
    }

    out_name =
        argument.substr(0, sep);

    out_index =
        argument.substr(sep + 1);

    return true;
}

// ─────────────────────────────────────────────────────────────
//  buildManipulationPayload
// ─────────────────────────────────────────────────────────────

nlohmann::json FindObject::buildManipulationPayload(
    const std::string& category,
    const std::string& index,
    const std::string& name,
    const geometry_msgs::msg::PoseStamped& pose) const
{
    const auto& p = pose.pose.position;
    const auto& q = pose.pose.orientation;

    nlohmann::json object;

    object["number"] = std::stoi(index);

    object["name"] = name;

    object["manipulation"] = {
        p.x,
        p.y,
        p.z,
        q.x,
        q.y,
        q.z,
        q.w
    };

    return {
        {"category", category},
        {"object", object}
    };
}

// ─────────────────────────────────────────────────────────────
//  publishFeedback
// ─────────────────────────────────────────────────────────────

void FindObject::publishFeedback(
    const std::shared_ptr<GoalHandle>& gh,
    FindObjectState state,
    const std::string& detail)
{
    if (!gh->is_active())
        return;

    std::string msg =
        "[find_object] " +
        stateName(state);

    if (!detail.empty())
        msg += ": " + detail;

    auto fb =
        std::make_shared<ExecuteTask::Feedback>();

    fb->feedback.message =
        msg;

    gh->publish_feedback(fb);

    RCLCPP_INFO(
        node_->get_logger(),
        "%s",
        msg.c_str());
}

// ─────────────────────────────────────────────────────────────
//  stateName
// ─────────────────────────────────────────────────────────────

std::string FindObject::stateName(
    FindObjectState s)
{
    switch (s)
    {
        case FindObjectState::READY:			return "READY";
        case FindObjectState::MOVING_HEAD:		return "MOVING_HEAD";
        case FindObjectState::STABILIZING:		return "STABILIZING";
        case FindObjectState::DETECTING:		return "DETECTING";
        case FindObjectState::LOOKING_AT_TARGET:return "LOOKING_AT_TARGET";
        case FindObjectState::UPDATING:			return "UPDATING";
        case FindObjectState::SUCCESS:			return "SUCCESS";
        case FindObjectState::FAILED:			return "FAILED";
        default:								return "UNKNOWN";
    }
}
