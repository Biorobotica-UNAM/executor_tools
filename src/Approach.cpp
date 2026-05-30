#include "executor_tools/Approach.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Approach::Approach(const rclcpp::Node::SharedPtr &node,
                   const std::shared_ptr<Hardware> &hardware,
                   const std::shared_ptr<Vision> &vision)
    : node_(node), hardware_(hardware), vision_(vision) {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// =========================================================
// run
// =========================================================
Result Approach::run(const std::string &argument,
                     const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active())
    return {false, "FAILED", "goal_handle_inactive"};

  publishFeedback(gh, ApproachState::READY, argument);

  // Resolve Target TF and type
  std::string target_frame = argument;
  bool is_person = false;

  if (argument == "person" || argument == "customer") {
    target_frame = "customer";
    is_person = true;
  } else {
    // If it does not start with OWL_ and isn't a raw frame name, resolve to
    // OWL_<arg>_0
    if (argument.find("OWL_") == std::string::npos && argument != "object") {
      target_frame = "OWL_" + argument + "_0";
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Approach starting: frame=%s (type=%s)",
              target_frame.c_str(), is_person ? "person" : "object");

  // =====================================================
  // LOOKING_FOR_TF
  // =====================================================
  publishFeedback(gh, ApproachState::LOOKING_FOR_TF, target_frame);

  bool tf_found = false;
  auto lookup_start = std::chrono::steady_clock::now();
  const auto kLookupTimeout = 5s;

  while (gh->is_active()) {
    try {
      // Quick check if transform is lookupable
      auto tf = tf_buffer_->lookupTransform("base_link", target_frame,
                                            tf2::TimePointZero, 100ms);
      tf_found = true;
      break;
    } catch (const tf2::TransformException &ex) {
      std::this_thread::sleep_for(100ms);
    }
    if (std::chrono::steady_clock::now() - lookup_start > kLookupTimeout) {
      break;
    }
  }

  if (!tf_found) {
    publishFeedback(gh, ApproachState::FAILED, "tf_not_found: " + target_frame);
    return {false, "FAILED", "tf_not_found"};
  }

  // =====================================================
  // APPROACHING LOOP
  // =====================================================
  publishFeedback(gh, ApproachState::APPROACHING);

  const double kTargetDist = 0.8; // 80 cm target distance
  const double kDistTol = 0.05;   // 5 cm distance tolerance
  const double kAngleTol = 0.1;   // ~5.7 degrees angular tolerance
  const double kYAlignTol = 0.03; // 3 cm sideways tolerance for objects

  // Proportional Gains
  const double kKpX = 0.6;
  const double kKpY = 0.6;
  const double kKpTheta = 0.8;

  // Safety Velocity Limits
  const double kMaxLinVel = 0.25; // m/s
  const double kMaxAngVel = 0.5;  // rad/s

  auto start_time = std::chrono::steady_clock::now();
  const auto kMaxDuration = 20s;

  while (gh->is_active()) {
    double tx = 0.0;
    double ty = 0.0;

    try {
      auto tf = tf_buffer_->lookupTransform("base_link", target_frame,
                                            tf2::TimePointZero, 100ms);
      tx = tf.transform.translation.x;
      ty = tf.transform.translation.y;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "Approach: TF lookup failed: %s",
                  ex.what());
      std::this_thread::sleep_for(50ms);
      continue;
    }

    // --- Person Target (Omnidirectional Approach) ---
    if (is_person) {
      double dist = std::sqrt(tx * tx + ty * ty);
      double angle = std::atan2(ty, tx);
      double dist_err = dist - kTargetDist;

      // Arrived check
      if (std::abs(dist_err) <= kDistTol && std::abs(angle) <= kAngleTol) {
        hardware_->MoveBase(0.0, 0.0, 0.0);
        publishFeedback(gh, ApproachState::SUCCESS);
        return {true, "SUCCESS", "approached_person"};
      }

      // Project errors on local base_link axes
      double ux = tx / dist;
      double uy = ty / dist;
      double err_x = dist_err * ux;
      double err_y = dist_err * uy;

      double vx = kKpX * err_x;
      double vy = kKpY * err_y;
      double wz = kKpTheta * angle;

      // Clamping velocities
      if (vx > kMaxLinVel)
        vx = kMaxLinVel;
      if (vx < -kMaxLinVel)
        vx = -kMaxLinVel;
      if (vy > kMaxLinVel)
        vy = kMaxLinVel;
      if (vy < -kMaxLinVel)
        vy = -kMaxLinVel;
      if (wz > kMaxAngVel)
        wz = kMaxAngVel;
      if (wz < -kMaxAngVel)
        wz = -kMaxAngVel;

      hardware_->MoveBase(vx, vy, wz);
    }
    // --- Object Target (Sideways Lateral Centering) ---
    else {
      double y_err = ty;

      // Sideways aligned check
      if (std::abs(y_err) <= kYAlignTol) {
        hardware_->MoveBase(0.0, 0.0, 0.0);
        publishFeedback(gh, ApproachState::SUCCESS);
        return {true, "SUCCESS", "aligned_object"};
      }

      double vy = kKpY * y_err;

      if (vy > kMaxLinVel)
        vy = kMaxLinVel;
      if (vy < -kMaxLinVel)
        vy = -kMaxLinVel;

      hardware_->MoveBase(0.0, vy, 0.0);
    }

    // Timeout check
    if (std::chrono::steady_clock::now() - start_time > kMaxDuration) {
      hardware_->MoveBase(0.0, 0.0, 0.0);
      publishFeedback(gh, ApproachState::FAILED, "timeout");
      return {false, "FAILED", "timeout"};
    }

    std::this_thread::sleep_for(50ms); // 20 Hz control loop
  }

  // Cancelled goal
  hardware_->MoveBase(0.0, 0.0, 0.0);
  publishFeedback(gh, ApproachState::FAILED, "goal_cancelled");
  return {false, "FAILED", "goal_cancelled"};
}

// =========================================================
// publishFeedback
// =========================================================
void Approach::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                               ApproachState state, const std::string &detail) {
  if (!gh->is_active())
    return;

  std::string msg = "[approach] " + stateName(state);
  if (!detail.empty())
    msg += ": " + detail;

  auto fb = std::make_shared<ExecuteTask::Feedback>();
  fb->feedback.message = msg;
  gh->publish_feedback(fb);

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string Approach::stateName(ApproachState s) {
  switch (s) {
  case ApproachState::READY:
    return "READY";
  case ApproachState::LOOKING_FOR_TF:
    return "LOOKING_FOR_TF";
  case ApproachState::APPROACHING:
    return "APPROACHING";
  case ApproachState::STABILIZING:
    return "STABILIZING";
  case ApproachState::SUCCESS:
    return "SUCCESS";
  case ApproachState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}
