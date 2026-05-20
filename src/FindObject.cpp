#include "executor_tools/FindObject.hpp"

#include <chrono>
#include <thread>
#include <algorithm>
#include <string>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
FindObject::FindObject(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<Hardware>& hardware,
    const std::shared_ptr<Vision>& vision)
: node_(node),
  hardware_(hardware),
  vision_(vision)
{
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


Result FindObject::run(
    const std::string& object_name,
    const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_inactive"};

    publishFeedback(gh, FindObjectState::READY, object_name);

    for (const auto& [pan, tilt] : scan_poses_)
    {
        if (!gh->is_active())
            return {false, "FAILED", "goal_cancelled"};

        // ─────────────────────────────────────────────────────
        //  MOVING_HEAD
        // ─────────────────────────────────────────────────────
        publishFeedback(gh, FindObjectState::MOVING_HEAD);

        hardware_->MoveHead({pan, tilt});

        std::this_thread::sleep_for(1500ms);

        // ─────────────────────────────────────────────────────
        //  STABILIZING
        // ─────────────────────────────────────────────────────
        publishFeedback(gh, FindObjectState::STABILIZING);

        std::this_thread::sleep_for(500ms);

        // ─────────────────────────────────────────────────────
        //  DETECTING
        // ─────────────────────────────────────────────────────
        publishFeedback(gh, FindObjectState::DETECTING);

        // ─────────────────────────────────────────────────────
        //  EMPTY CHAIR  (caso especial, sin TF2)
        // ─────────────────────────────────────────────────────
        if (object_name == "empty_chair")
        {
            geometry_msgs::msg::PointStamped seat_point;

            if (detectEmptyChair(seat_point))
            {
                publishFeedback(gh, FindObjectState::LOOKING_AT_TARGET);
                hardware_->LookAt(seat_point);
                publishFeedback(gh, FindObjectState::SUCCESS, "empty_chair");

                return {true, "SUCCESS", "empty_chair_found"};
            }
        }

        else
        {
            geometry_msgs::msg::PoseStamped object_pose;

            if (detectWithOWL(object_name, object_pose))
            {
                publishFeedback(gh, FindObjectState::LOOKING_AT_TARGET);
                hardware_->LookAt(object_pose);
                publishFeedback(gh, FindObjectState::SUCCESS, object_name);

                return {true, "SUCCESS", object_name + "_found"};
            }
        }
    }

    publishFeedback(gh, FindObjectState::FAILED, object_name);
    return {false, "FAILED", object_name + "_not_found"};
}

bool FindObject::detectWithOWL(
    const std::string& object_name,
    geometry_msgs::msg::PoseStamped& pose)
{
    //  servicio DetectOwlV2
    auto result = vision_->DetectOwlV2({object_name});

    if (!result.first || result.second.empty())
        return false;

    // Normalizar el label

    std::string label = result.second[0].class_name;
    std::replace(label.begin(), label.end(), ' ', '_');

    const rclcpp::Time now_time = node_->get_clock()->now();
    std::string                          best_frame;
    double                               best_age = 9999.0;
    geometry_msgs::msg::TransformStamped best_tf;

    for (int i = 0; i < MAX_OWL_INSTANCES; ++i)
    {
        const std::string candidate = "OWL_" + label + "_" + std::to_string(i);
        try
        {
            auto tf_stamped = tf_buffer_->lookupTransform(
                "base_link", candidate,
                tf2::TimePointZero,
                tf2::durationFromSec(0.1));   // timeout 100 ms

            double age = std::abs(
                now_time.seconds() -
                rclcpp::Time(tf_stamped.header.stamp).seconds());

            if (age < best_age)
            {
                best_age   = age;
                best_frame = candidate;
                best_tf    = tf_stamped;
            }
        }
        catch (...)
        {
            break;
        }
    }

    if (best_frame.empty())
    {
        RCLCPP_WARN(node_->get_logger(),
            "[find_object] No TF encontrada para '%s'", label.c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
        "[find_object] TF encontrada: %s  (edad: %.4f s)",
        best_frame.c_str(), best_age);

    if (best_age > MAX_TF_AGE_S)
    {
        RCLCPP_WARN(node_->get_logger(),
            "[find_object] TF obsoleta para '%s' (%.2f s). Saltando pose.",
            best_frame.c_str(), best_age);
        return false;
    }

    // PoseStamped en base_link
    pose.header.frame_id       = "base_link";
    pose.header.stamp.sec      = 0;    // stamp=0 evita errores de extrapolación en MoveIt
    pose.header.stamp.nanosec  = 0;

    pose.pose.position.x  = best_tf.transform.translation.x;
    pose.pose.position.y  = best_tf.transform.translation.y;
    pose.pose.position.z  = best_tf.transform.translation.z;
    pose.pose.orientation = best_tf.transform.rotation;

    // altura mínima de seguridad
    if (pose.pose.position.z < MIN_Z_THRESHOLD)
        pose.pose.position.z = MIN_Z_HEIGHT;

    RCLCPP_INFO(node_->get_logger(),
        "[find_object] Objetivo '%s' en [%.2f, %.2f, %.2f]",
        label.c_str(),
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z);

    return true;
}

// ─────────────────────────────────────────────────────────────
//  detectEmptyChair
// ─────────────────────────────────────────────────────────────
bool FindObject::detectEmptyChair(
    geometry_msgs::msg::PointStamped& point)
{
    auto result = vision_->FindSeat();

    if (!result.first)
        return false;

    point = result.second;
    return true;
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

    std::string msg = "[find_object] " + stateName(state);

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
std::string FindObject::stateName(FindObjectState s)
{
    switch (s)
    {
        case FindObjectState::READY:            return "READY";
        case FindObjectState::MOVING_HEAD:      return "MOVING_HEAD";
        case FindObjectState::STABILIZING:      return "STABILIZING";
        case FindObjectState::DETECTING:        return "DETECTING";
        case FindObjectState::LOOKING_AT_TARGET:return "LOOKING_AT_TARGET";
        case FindObjectState::SUCCESS:          return "SUCCESS";
        case FindObjectState::FAILED:           return "FAILED";
        default:                                return "UNKNOWN";
    }
}