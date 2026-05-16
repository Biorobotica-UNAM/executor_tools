#include "executor_tools/Point.hpp"

#include <cmath>
#include <chrono>
#include <thread>

Point::Point(const rclcpp::Node::SharedPtr& node,
                   const std::shared_ptr<Hardware>& hardware)
    : node_(node),
      hardware_(hardware),
      tf_buffer_(node_->get_clock()),
      tf_listener_(tf_buffer_)
{
    use_takeshi_ = node_->declare_parameter("use_takeshi", false);

    if (!use_takeshi_) {
        camera_frame_ = "head_link";
        base_frame_   = "base_link";
    } else {
        camera_frame_ = "head_pan_link";
        base_frame_   = "base_link";
    }

    RCLCPP_INFO(node_->get_logger(), "Point component initialized (using Hardware).");
}

Result Point::run(const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    publishFeedback(gh, PointState::READY);
    publishFeedback(gh, PointState::ALIGNING_BASE);

    rclcpp::Rate rate(30.0);
    try {
        auto tf = tf_buffer_.lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);
        double theta = tf2::getYaw(tf.transform.rotation);

        if (std::abs(theta) > 1e-3) {
            if (!hardware_->MoveHead({0.0, 0.0}, std::chrono::seconds(2))) {
                publishFeedback(gh, PointState::FAILED, "failed to move head");
                return {false, "FAILED", "head move failed"};
            }

            double angular_speed = 0.6;
            double duration = std::abs(theta / angular_speed) + 0.3;
            double angular_z = (theta < 0.0) ? -angular_speed : angular_speed;

            auto start = node_->now();
            while (rclcpp::ok() && (node_->now() - start).seconds() < duration) {
                if (!gh->is_active()) {
                    hardware_->MoveBase(0.0, 0.0, 0.0);
                    publishFeedback(gh, PointState::FAILED, "cancelled");
                    return {false, "CANCELLED", "Point cancelled during base rotation"};
                }
                hardware_->MoveBase(0.0, 0.0, angular_z);
                rate.sleep();
            }
        }

        hardware_->MoveBase(0.0, 0.0, 0.0);

        publishFeedback(gh, PointState::MOVING_HEAD);
        if (!hardware_->MoveHead({0.0, 0.0}, std::chrono::seconds(2))) {
            publishFeedback(gh, PointState::FAILED, "head final move failed");
            return {false, "FAILED", "head final move failed"};
        }

        publishFeedback(gh, PointState::POINTING_ARM);
        std::vector<double> arm_pose;
        if (use_takeshi_) {
            arm_pose = {-0.25, 0.0, -1.5, 0.0};
        } else {
            arm_pose = {0.6, 0.1, 0.3, 1.2, 0.0, 0.0};
        }
        if (!hardware_->MoveLeftArm(arm_pose, 20, std::chrono::seconds(15))) {
            publishFeedback(gh, PointState::FAILED, "arm move failed");
            return {false, "FAILED", "arm move failed"};
        }

        publishFeedback(gh, PointState::SUCCESS);
        return {true, "SUCCESS", "Point motion completed"};

    } catch (const tf2::TransformException& ex) {
        hardware_->MoveBase(0.0, 0.0, 0.0);
        std::string err = std::string("TF error: ") + ex.what();
        publishFeedback(gh, PointState::FAILED, err);
        return {false, "FAILED", err};
    }
}

Result Point::disable(const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    publishFeedback(gh, PointState::READY, "disabling Point");
    hardware_->MoveBase(0.0, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::vector<double> reset_pose;
    if (use_takeshi_) {
        reset_pose = {0.0, -1.57, -1.57, 0.0};
    } else {
        reset_pose = {0.0, 0.0, 0.3, 1.5, 0.0, 1.2};
    }
    if (!hardware_->MoveLeftArm(reset_pose, 20, std::chrono::seconds(15))) {
        publishFeedback(gh, PointState::FAILED, "arm reset failed");
        return {false, "FAILED", "arm reset failed"};
    }

    publishFeedback(gh, PointState::SUCCESS, "Point disabled");
    return {true, "SUCCESS", "Point disabled"};
}

void Point::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                               PointState state,
                               const std::string& detail)
{
    if (!gh || !gh->is_active()) return;

    std::string msg = "[Point] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<executor_interfaces::action::ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

std::string Point::stateName(PointState s)
{
    switch (s) {
        case PointState::READY:         return "READY";
        case PointState::ALIGNING_BASE: return "ALIGNING_BASE";
        case PointState::MOVING_HEAD:   return "MOVING_HEAD";
        case PointState::POINTING_ARM:  return "POINTING_ARM";
        case PointState::SUCCESS:       return "SUCCESS";
        case PointState::FAILED:        return "FAILED";
        default:                           return "UNKNOWN";
    }
}