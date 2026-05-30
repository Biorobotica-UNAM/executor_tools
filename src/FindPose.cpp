#include "executor_tools/FindPose.hpp"
#include "executor_tools/Result.hpp"

#include <chrono>
#include <thread>
#include <string>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────

FindPose::FindPose(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<Hardware>& hardware,
    const std::shared_ptr<HRI>& hri,
    const std::shared_ptr<Knowledge>& knowledge)
: node_(node),
  hardware_(hardware),
  hri_(hri),
  knowledge_(knowledge)
{}

// ─────────────────────────────────────────────────────────────
//  run
// ─────────────────────────────────────────────────────────────

Result FindPose::run(
    const std::string& argument,
    const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_inactive"};

    // ── READY ──────────────────────────────────────────────

    publishFeedback(
        gh,
        FindPoseState::READY,
        argument);

    if (argument.empty())
    {
        publishFeedback(
            gh,
            FindPoseState::FAILED,
            "empty_argument");

        return {
            false,
            "FAILED",
            "empty_argument"
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
            FindPoseState::MOVING_HEAD);

        hardware_->MoveHead({pan, tilt});

        std::this_thread::sleep_for(1500ms);

        // ── STABILIZING ────────────────────────────────────

        publishFeedback(
            gh,
            FindPoseState::STABILIZING);

        std::this_thread::sleep_for(500ms);

        // ── DETECTING ──────────────────────────────────────

        publishFeedback(
            gh,
            FindPoseState::DETECTING,
            argument);

        std::string out_direction;
        geometry_msgs::msg::PointStamped out_point;

        // Pasamos "any" en el arm_side para que el script de Python evalúe cualquier brazo disponible
        bool success = hri_->FindPose(argument, "any", out_direction, out_point);

        if (success)
        {
            // ── LOOKING_AT_TARGET ─────────────────────

            publishFeedback(
                gh,
                FindPoseState::LOOKING_AT_TARGET,
                out_direction);

            hardware_->LookAt(out_point);
            std::this_thread::sleep_for(1500ms);

            // ── UPDATING ─────────────────────────────

            publishFeedback(
                gh,
                FindPoseState::UPDATING,
                out_direction);

            // Estructuramos el payload en formato JSON para el sistema de Knowledge de BioRobotics
            const auto payload =
                buildHumanPosePayload(
                    argument,
                    out_direction,
                    out_point);

            const bool stored =
                knowledge_->update_world_state(payload);

            if (!stored)
            {
                publishFeedback(
                    gh,
                    FindPoseState::FAILED,
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
                FindPoseState::SUCCESS,
                out_direction);

            return {
                true,
                "SUCCESS",
                "human_" + argument + "_found_at_" + out_direction
            };
        }
    }

    publishFeedback(
        gh,
        FindPoseState::FAILED,
        argument);

    return {
        false,
        "FAILED",
        argument + "_pose_not_found"
    };
}

// ─────────────────────────────────────────────────────────────
//  buildHumanPosePayload
// ─────────────────────────────────────────────────────────────

nlohmann::json FindPose::buildHumanPosePayload(
    const std::string& action_type,
    const std::string& direction,
    const geometry_msgs::msg::PointStamped& point) const
{
    nlohmann::json human_data;

    human_data["action"] = action_type;
    human_data["direction_zone"] = direction;
    human_data["frame_id"] = point.header.frame_id;
    
    // Almacenamos las coordenadas 3D para que la base móvil o el brazo sepan a dónde aproximarse
    human_data["position"] = {
        point.point.x,
        point.point.y,
        point.point.z
    };

    return {
        {"category", "detected_human"},
        {"pose_info", human_data}
    };
}

// ─────────────────────────────────────────────────────────────
//  publishFeedback
// ─────────────────────────────────────────────────────────────

void FindPose::publishFeedback(
    const std::shared_ptr<GoalHandle>& gh,
    FindPoseState state,
    const std::string& detail)
{
    if (!gh->is_active())
        return;

    std::string msg =
        "[find_pose] " +
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

std::string FindPose::stateName(
    FindPoseState s)
{
    switch (s)
    {
        case FindPoseState::READY:             return "READY";
        case FindPoseState::MOVING_HEAD:       return "MOVING_HEAD";
        case FindPoseState::STABILIZING:       return "STABILIZING";
        case FindPoseState::DETECTING:         return "DETECTING";
        case FindPoseState::LOOKING_AT_TARGET: return "LOOKING_AT_TARGET";
        case FindPoseState::UPDATING:          return "UPDATING";
        case FindPoseState::SUCCESS:           return "SUCCESS";
        case FindPoseState::FAILED:            return "FAILED";
        default:                               return "UNKNOWN";
    }
}