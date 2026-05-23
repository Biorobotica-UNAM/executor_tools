#include "executor_tools/Align.hpp"

#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Align::Align(const rclcpp::Node::SharedPtr &node,
             const std::shared_ptr<Hardware> &hardware,
             const std::shared_ptr<Vision> &vision)
    : node_(node), hardware_(hardware), vision_(vision) {}

// =========================================================
// run
// =========================================================
Result Align::run(const std::string &argument,
                  const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active())
    return {false, "FAILED", "goal_handle_inactive"};

  publishFeedback(gh, AlignState::READY, argument);

  publishFeedback(gh, AlignState::MOVING_TORSO);
  bool torso_moved = hardware_->MoveTorso(0.3);
  if (!torso_moved) {
    publishFeedback(gh, AlignState::FAILED, "move_torso_failed");
    return {false, "FAILED", "move_torso_failed"};
  }

  publishFeedback(gh, AlignState::MOVING_HEAD);
  bool moved = hardware_->MoveHead({0.0, -0.6});
  if (!moved) {
    publishFeedback(gh, AlignState::FAILED, "move_head_failed");
    return {false, "FAILED", "move_head_failed"};
  }

  const double kAngleTolerance = 2.0;
  const double kAngularSpeed = 0.2;
  const auto kMaxDuration = 15s;

  auto start_time = std::chrono::steady_clock::now();

  double last_angular_velocity = 0.0;
  while (gh->is_active()) {
    double angle = vision_->GetTableAlignment();

    publishFeedback(gh, AlignState::GET_ALIGNMENT, std::to_string(angle));

    if (std::fabs(angle) <= kAngleTolerance) {
      // Dar un último pulso en la misma dirección para compensar
      if (last_angular_velocity != 0.0) {
        double pulse_velocity = (last_angular_velocity > 0) ? 0.3 : -0.3;
        RCLCPP_INFO(node_->get_logger(),
                    "pulso de compensacion: %.2f",
                    pulse_velocity);
        hardware_->MoveBase(0.0, 0.0, pulse_velocity);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(300)); // pulso de 300ms
      }

      hardware_->MoveBase(0.0, 0.0, 0.0);
      publishFeedback(gh, AlignState::SUCCESS);
      return {true, "SUCCESS", "aligned"};
    }

    // Controlador Proporcional para evitar oscilaciones ("baile")
    double kp = 0.03;
    double angular = kp * angle; // ¡El signo debe ser positivo!

    // Limitar la velocidad máxima
    if (angular > kAngularSpeed)
      angular = kAngularSpeed;
    if (angular < -kAngularSpeed)
      angular = -kAngularSpeed;

    // Establecer una velocidad mínima para vencer la fricción
    double min_speed = 0.05;
    if (angular > 0 && angular < min_speed)
      angular = min_speed;
    if (angular < 0 && angular > -min_speed)
      angular = -min_speed;

    last_angular_velocity = angular;

    // Log the angular velocity command
    RCLCPP_INFO(node_->get_logger(),
                "Ang error: %.2f, Rotating with angular velocity: %.2f", angle,
                angular);

    hardware_->MoveBase(0.0, 0.0, angular);
    publishFeedback(gh, AlignState::ROTATING);

    std::this_thread::sleep_for(100ms);

    /*
    if (std::chrono::steady_clock::now() - start_time > kMaxDuration) {
        hardware_->MoveBase(0.0, 0.0, 0.0);
        publishFeedback(gh, AlignState::FAILED, "timeout");
        return {false, "FAILED", "timeout"};
    }
    */
  }

  // goal cancelled
  hardware_->MoveBase(0.0, 0.0, 0.0);
  publishFeedback(gh, AlignState::FAILED, "goal_cancelled");
  return {false, "FAILED", "goal_cancelled"};
}

// =========================================================
// publishFeedback
// =========================================================
void Align::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                            AlignState state, const std::string &detail) {
  if (!gh->is_active())
    return;

  std::string msg = "[align] " + stateName(state);
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
std::string Align::stateName(AlignState s) {
  switch (s) {
  case AlignState::READY:
    return "READY";
  case AlignState::MOVING_HEAD:
    return "MOVING_HEAD";
  case AlignState::MOVING_TORSO:
    return "MOVING_TORSO";
  case AlignState::GET_ALIGNMENT:
    return "GET_ALIGNMENT";
  case AlignState::ROTATING:
    return "ROTATING";
  case AlignState::STABILIZING:
    return "STABILIZING";
  case AlignState::SUCCESS:
    return "SUCCESS";
  case AlignState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}
