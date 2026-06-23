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
  if (!hardware_->MoveTorso(0.3)) {
    publishFeedback(gh, AlignState::FAILED, "move_torso_failed");
    return {false, "FAILED", "move_torso_failed"};
  }

  publishFeedback(gh, AlignState::MOVING_HEAD);
  if (!hardware_->MoveHead({0.0, -0.6})) {
    publishFeedback(gh, AlignState::FAILED, "move_head_failed");
    return {false, "FAILED", "move_head_failed"};
  }

  // ── Parámetros del controlador ──────────────────────────────────────────
  // Tolerancia alcanzable para una base móvil real.
  const double kAngleTolerance = 1.5;   // grados

  // Velocidades en rad/s.
  // kAngularSpeedMax: techo para errores grandes.
  // kAngularSpeedMin: piso para vencer fricción estática.
  //   Con ciclo de 50 ms: paso máximo = 0.04 × 0.05 s = 0.002 rad ≈ 0.11°
  //   << kAngleTolerance → no puede producir overshoot de más de 0.11°
  //   desde fuera del deadband en un solo tick.
  const double kAngularSpeedMax = 0.20;
  const double kAngularSpeedMin = 0.04;

  // Ganancia proporcional (grados → rad/s).
  // Con error = 7°: angular = 0.025 × 7 = 0.175 rad/s (bajo el techo).
  const double kKp = 0.025;

  // Muestras consecutivas dentro de tolerancia para confirmar alineación.
  // Filtra falsos positivos de visión (ruido de un solo frame).
  const int kStableRequired = 3;

  const auto kMaxDuration = 30s;

  // 20 Hz → 50 ms/ciclo. Un único MoveBase por muestra de visión → lazo cerrado.
  rclcpp::Rate rate(20.0);

  const bool align_to_object = (argument == "container");
  auto start_time = std::chrono::steady_clock::now();
  int stable_count = 0;

  while (gh->is_active()) {

    // ── Timeout ─────────────────────────────────────────────────────────────
    if (std::chrono::steady_clock::now() - start_time > kMaxDuration) {
      hardware_->MoveBase(0.0, 0.0, 0.0);
      publishFeedback(gh, AlignState::FAILED, "timeout");
      return {false, "FAILED", "timeout"};
    }

    // ── Medición del error angular ──────────────────────────────────────────
    double angle = 0.0;

    if (align_to_object) {
      geometry_msgs::msg::PoseStamped object_pose, object_nav;
      std::string detected_name;

      if (!vision_->GetOwlPose("container", object_pose, object_nav,
                               detected_name)) {
        RCLCPP_WARN(node_->get_logger(),
                    "[Align] No se detecto el contenedor. Reintentando...");
        hardware_->MoveBase(0.0, 0.0, 0.0);
        stable_count = 0;
        rate.sleep();
        continue;
      }

      angle = std::atan2(object_pose.pose.position.y,
                         object_pose.pose.position.x) *
              180.0 / M_PI;
    } else {
      bool is_round_table = (argument == "round_table" || argument == "round");
      angle = vision_->GetTableAlignment(is_round_table);
    }

    publishFeedback(gh, AlignState::GET_ALIGNMENT, std::to_string(angle));

    RCLCPP_INFO(node_->get_logger(),
                "[Align] error=%.2f°  stable=%d/%d",
                angle, stable_count, kStableRequired);

    // ── Verificación de convergencia ────────────────────────────────────────
    if (std::fabs(angle) <= kAngleTolerance) {
      ++stable_count;
      hardware_->MoveBase(0.0, 0.0, 0.0); // mantener quieto mientras se confirma
      if (stable_count >= kStableRequired) {
        publishFeedback(gh, AlignState::SUCCESS);
        return {true, "SUCCESS", "aligned"};
      }
      rate.sleep();
      continue;
    }

    // Error fuera de tolerancia → reiniciar conteo de estabilidad
    stable_count = 0;

    // ── Control Proporcional ────────────────────────────────────────────────
    double angular = kKp * angle;

    // Clamp [min, max] preservando signo
    double abs_w = std::fabs(angular);
    abs_w = std::min(abs_w, kAngularSpeedMax);
    abs_w = std::max(abs_w, kAngularSpeedMin);
    angular = std::copysign(abs_w, angle);

    RCLCPP_INFO(node_->get_logger(),
                "[Align] angular_cmd=%.3f rad/s", angular);

    publishFeedback(gh, AlignState::ROTATING);

    // Un único comando por muestra de visión → lazo realmente cerrado.
    // El base controller mantiene la velocidad entre ticks (50 ms es suficiente).
    hardware_->MoveBase(0.0, 0.0, angular);
    rate.sleep();
  }

  // Goal cancelado externamente
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
  case AlignState::READY:       return "READY";
  case AlignState::MOVING_HEAD: return "MOVING_HEAD";
  case AlignState::MOVING_TORSO:return "MOVING_TORSO";
  case AlignState::GET_ALIGNMENT:return "GET_ALIGNMENT";
  case AlignState::ROTATING:    return "ROTATING";
  case AlignState::STABILIZING: return "STABILIZING";
  case AlignState::SUCCESS:     return "SUCCESS";
  case AlignState::FAILED:      return "FAILED";
  default:                      return "UNKNOWN";
  }
}
