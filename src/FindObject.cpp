#include "executor_tools/FindObject.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

FindObject::FindObject(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<Hardware>& hardware,
    const std::shared_ptr<Vision>& vision)
: node_(node),
  hardware_(hardware),
  vision_(vision)
{
    tf_buffer_ =
        std::make_shared<tf2_ros::Buffer>(
            node_->get_clock());

    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_);
}

FindObject::Result FindObject::run(
    const std::string& object_name,
    const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_inactive"};

    publishFeedback(
        gh,
        FindObjectState::READY,
        object_name);

    for (const auto& [pan, tilt] : scan_poses_)
    {
        if (!gh->is_active())
            return {false, "FAILED", "goal_cancelled"};

        // =====================================================
        // MOVING_HEAD
        // =====================================================

        publishFeedback(
            gh,
            FindObjectState::MOVING_HEAD);

        hardware_->MoveHead({pan, tilt});

        std::this_thread::sleep_for(1500ms);

        // =====================================================
        // STABILIZING
        // =====================================================

        publishFeedback(
            gh,
            FindObjectState::STABILIZING);

        std::this_thread::sleep_for(500ms);

        // =====================================================
        // DETECTING
        // =====================================================

        publishFeedback(
            gh,
            FindObjectState::DETECTING);

        // =====================================================
        // EMPTY CHAIR
        // =====================================================

        if (object_name == "empty_chair")
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

        // =====================================================
        // NORMAL OBJECTS
        // =====================================================

        else
        {
            geometry_msgs::msg::PoseStamped object_pose;

            if (detectWithOWL(
                    object_name,
                    object_pose))
            {
                publishFeedback(
                    gh,
                    FindObjectState::LOOKING_AT_TARGET);

                hardware_->LookAt(object_pose);

                publishFeedback(
                    gh,
                    FindObjectState::SUCCESS,
                    object_name);

                return {
                    true,
                    "SUCCESS",
                    object_name + "_found"
                };
            }
        }
    }

    publishFeedback(
        gh,
        FindObjectState::FAILED,
        object_name);

    return {
        false,
        "FAILED",
        object_name + "_not_found"
    };
}

bool FindObject::detectWithOWL(
    const std::string& object_name,
    geometry_msgs::msg::PoseStamped& pose)
{
    auto result =
        vision_->DetectOwlV2({object_name});

    if (!result.first)
        return false;

    if (result.second.empty())
        return false;

    auto detection = result.second[0];

    std::string label =
        detection.class_name;

    std::replace(
        label.begin(),
        label.end(),
        ' ',
        '_');

    try
    {
        auto tf =
            tf_buffer_->lookupTransform(
                "base_link",
                "OWL_" + label + "_0",
                tf2::TimePointZero,
                200ms);

        pose.header =
            tf.header;

        pose.header.frame_id =
            "base_link";

        pose.pose.position.x =
            tf.transform.translation.x;

        pose.pose.position.y =
            tf.transform.translation.y;

        pose.pose.position.z =
            tf.transform.translation.z;

        pose.pose.orientation =
            tf.transform.rotation;

        RCLCPP_INFO(
            node_->get_logger(),
            "Object found [%s] at [%.2f %.2f %.2f]",
            label.c_str(),
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z);

        return true;
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "TF lookup failed for [%s]: %s",
            label.c_str(),
            ex.what());

        return false;
    }
}

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

void FindObject::publishFeedback(
    const std::shared_ptr<GoalHandle>& gh,
    FindObjectState state,
    const std::string& detail)
{
    if (!gh->is_active())
        return;

    std::string msg =
        "[find_object] " + stateName(state);

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

std::string FindObject::stateName(
    FindObjectState s)
{
    switch (s)
    {
        case FindObjectState::READY:
            return "READY";

        case FindObjectState::MOVING_HEAD:
            return "MOVING_HEAD";

        case FindObjectState::STABILIZING:
            return "STABILIZING";

        case FindObjectState::DETECTING:
            return "DETECTING";

        case FindObjectState::LOOKING_AT_TARGET:
            return "LOOKING_AT_TARGET";

        case FindObjectState::SUCCESS:
            return "SUCCESS";

        case FindObjectState::FAILED:
            return "FAILED";

        default:
            return "UNKNOWN";
    }
}
