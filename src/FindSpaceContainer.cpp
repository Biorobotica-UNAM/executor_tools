#include "executor_tools/FindSpace.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <thread>
#include <unordered_map>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

bool FindSpace::precenterContainerWithOwl(
    const std::shared_ptr<GoalHandle> &gh,
    const SpaceTarget &space_target) {
  constexpr int kMaxHeadCorrections = 4;
  constexpr int kMaxBaseCorrections = 7;
  constexpr double kIdealXMin = 0.75;
  constexpr double kIdealXMax = 0.85;
  constexpr double kIdealAbsYMax = 0.22;
  constexpr double kScanXMin = 0.65;
  constexpr double kScanXMax = 1.60;
  constexpr double kScanAbsYMax = 0.65;
  constexpr double kActionXMin = 0.45;
  constexpr double kActionXMax = 2.00;
  constexpr double kActionAbsYMax = 1.10;
  constexpr double kImageCenterX = 320.0;
  constexpr double kImageCenterTolerancePx = 55.0;
  constexpr double kHeadPanMax = 0.55;
  constexpr double kHeadPanMin = -0.55;
  constexpr double kHeadTiltMin = -0.95;
  constexpr double kHeadTiltMax = -0.35;
  constexpr double kMaxVx = 0.10;
  constexpr double kMaxVy = 0.07;
  constexpr double kMaxWz = 0.18;

  struct SearchStep {
    double pan;
    double tilt_offset;
    double vx;
    double vy;
    double wz;
    int pulse_ms;
    const char *label;
  };

  const std::array<SearchStep, 14> search_steps = {{
      {0.0, 0.0, 0.0, 0.0, 0.0, 0, "centro"},
      {-0.35, 0.0, 0.0, 0.0, 0.0, 0, "derecha"},
      {0.35, 0.0, 0.0, 0.0, 0.0, 0, "izquierda"},
      {0.0, -0.14, 0.0, 0.0, 0.0, 0, "abajo"},
      {-0.35, -0.14, 0.0, 0.0, 0.0, 0, "abajo-derecha"},
      {0.35, -0.14, 0.0, 0.0, 0.0, 0, "abajo-izquierda"},
      {0.0, 0.16, 0.0, 0.0, 0.0, 0, "arriba"},
      {-0.35, 0.16, 0.0, 0.0, 0.0, 0, "arriba-derecha"},
      {0.35, 0.16, 0.0, 0.0, 0.0, 0, "arriba-izquierda"},
      {0.0, 0.0, 0.0, 0.0, kMaxWz, 650, "giro-izquierda"},
      {0.0, 0.0, 0.0, 0.0, -2.0 * kMaxWz, 1200, "giro-derecha"},
      {0.0, 0.0, 0.0, 0.0, kMaxWz, 650, "regresa-orientacion"},
      {0.0, 0.0, kMaxVx * 0.8, 0.0, 0.0, 650, "adelante"},
      {0.0, 0.0, -kMaxVx * 0.7, 0.0, 0.0, 600, "atras"},
  }};

  double commanded_pan = 0.0;
  double commanded_tilt = space_target.head_pitch;
  int head_corrections = 0;
  int base_corrections = 0;
  int search_step_index = 0;
  // Esta etapa no decide el punto de drop. Solo intenta poner al robot/camara
  // en una postura donde el detector geometrico vea suficiente nube util.
  for (int attempt = 0;
       attempt <= kMaxHeadCorrections + kMaxBaseCorrections +
                      static_cast<int>(search_steps.size()) + 1;
       ++attempt) {
    if (!gh->is_active()) {
      return false;
    }

    publishFeedback(
        gh, FindSpaceState::PRECENTER_CONTAINER_WITH_OWL,
        "centrando contenedor con cabeza/base y OWLv2 intento=" +
            std::to_string(attempt + 1));

    OwlContainerPose pose;
    std::string error;
    if (!requestOwlContainerPose(pose, error)) {
      if (search_step_index < static_cast<int>(search_steps.size())) {
        const auto &step = search_steps[search_step_index++];
        commanded_pan = step.pan;
        commanded_tilt =
            std::clamp(space_target.head_pitch + step.tilt_offset,
                       kHeadTiltMin, kHeadTiltMax);
        RCLCPP_WARN(node_->get_logger(),
                    "[FindSpace] precenter OWL sin deteccion (%s); "
                    "busqueda activa paso=%d/%zu %s head=(%.3f %.3f) "
                    "base=(%.3f %.3f %.3f) duracion=%.2fs",
                    error.c_str(), search_step_index, search_steps.size(),
                    step.label, commanded_pan, commanded_tilt, step.vx,
                    step.vy, step.wz,
                    static_cast<double>(step.pulse_ms) / 1000.0);
        if (!hardware_->MoveHead({commanded_pan, commanded_tilt})) {
          RCLCPP_WARN(node_->get_logger(),
                      "[FindSpace] fallo al mover cabeza en busqueda activa");
        }
        if (step.pulse_ms > 0) {
          publishBasePulse(step.vx, step.vy, step.wz,
                           std::chrono::milliseconds(step.pulse_ms));
        }
        std::this_thread::sleep_for(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(
                    space_target.sensor_settle_delay_sec)));
        continue;
      }

      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] precenter OWL no disponible/sin deteccion tras "
                  "busqueda activa: %s; se usara fallback geometrico",
                  error.c_str());
      return true;
    }

    const bool in_scan_zone =
        pose.x >= kScanXMin && pose.x <= kScanXMax &&
        std::fabs(pose.y) <= kScanAbsYMax;
    const bool in_ideal_zone =
        pose.x >= kIdealXMin && pose.x <= kIdealXMax &&
        std::fabs(pose.y) <= kIdealAbsYMax;
    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] precenter OWL class=%s frame=%s conf=%.2f "
                "base=(%.3f %.3f %.3f) bbox_cx=%.1f bbox_size=(%.1f %.1f) "
                "ideal_x=[%.2f %.2f] ideal_abs_y<=%.2f "
                "scan_x=[%.2f %.2f] scan_abs_y<=%.2f",
                pose.class_name.c_str(), pose.frame.c_str(), pose.confidence,
                pose.x, pose.y, pose.z, pose.bbox_center_x, pose.bbox_width,
                pose.bbox_height, kIdealXMin, kIdealXMax, kIdealAbsYMax,
                kScanXMin, kScanXMax, kScanAbsYMax);
    // DEBUG: este log es la referencia principal para saber si el problema es
    // de vista (bbox_cx), distancia (x) o lateralidad (y).

    if (!in_scan_zone) {
      const bool in_action_zone =
          pose.x >= kActionXMin && pose.x <= kActionXMax &&
          std::fabs(pose.y) <= kActionAbsYMax;
      if (in_action_zone && base_corrections < kMaxBaseCorrections) {
        double vx = 0.0;
        double vy = 0.0;
        double wz = 0.0;
        if (pose.x > kIdealXMax) {
          vx = kMaxVx;
        } else if (pose.x < kIdealXMin) {
          vx = -0.8 * kMaxVx;
        }
        if (pose.y > kIdealAbsYMax) {
          vy = kMaxVy;
          wz = kMaxWz;
        } else if (pose.y < -kIdealAbsYMax) {
          vy = -kMaxVy;
          wz = -kMaxWz;
        }

        const double x_error = std::max(
            {0.0, pose.x - kIdealXMax, kIdealXMin - pose.x});
        const double y_error =
            std::max(0.0, std::fabs(pose.y) - kIdealAbsYMax);
        const auto pulse_ms = static_cast<int>(std::clamp(
            500.0 + 850.0 * std::max(x_error, y_error), 500.0, 1100.0));

        ++base_corrections;
        RCLCPP_INFO(node_->get_logger(),
                    "[FindSpace] contenedor fuera de zona de escaneo pero "
                    "corregible; base vx=%.3f vy=%.3f wz=%.3f duracion=%.2fs "
                    "base_correction=%d/%d",
                    vx, vy, wz, static_cast<double>(pulse_ms) / 1000.0,
                    base_corrections, kMaxBaseCorrections);
        publishBasePulse(vx, vy, wz, std::chrono::milliseconds(pulse_ms));
        commanded_pan = 0.0;
        commanded_tilt = space_target.head_pitch;
        hardware_->MoveHead({commanded_pan, commanded_tilt});
        std::this_thread::sleep_for(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(
                    space_target.sensor_settle_delay_sec)));
        continue;
      }

      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] contenedor fuera de la zona corregible actual: "
                  "base=(%.3f %.3f %.3f) scan_x=[%.2f %.2f] abs_y<=%.2f "
                  "action_x=[%.2f %.2f] action_abs_y<=%.2f. "
                  "Se requiere reaproximacion externa.",
                  pose.x, pose.y, pose.z, kScanXMin, kScanXMax,
                  kScanAbsYMax, kActionXMin, kActionXMax, kActionAbsYMax);
      return false;
    }

    const double image_error =
        pose.bbox_center_x > 0.0 ? pose.bbox_center_x - kImageCenterX : 0.0;
    const double desired_pan =
        std::clamp(std::atan2(pose.y, std::max(0.05, pose.x)), kHeadPanMin,
                   kHeadPanMax);
    const bool image_centered =
        std::fabs(image_error) <= kImageCenterTolerancePx;
    const bool pan_close = std::fabs(desired_pan - commanded_pan) <= 0.05;
    if (in_ideal_zone && image_centered && pan_close) {
      RCLCPP_INFO(node_->get_logger(),
                  "[FindSpace] precenter OWL listo: contenedor en zona ideal "
                  "para procesar nube");
      return true;
    }

    if ((!image_centered || !pan_close) &&
        head_corrections < kMaxHeadCorrections) {
      commanded_pan = desired_pan;
      commanded_tilt = space_target.head_pitch;
      ++head_corrections;
      RCLCPP_INFO(node_->get_logger(),
                  "[FindSpace] precenter moviendo cabeza pan=%.3f tilt=%.3f "
                  "image_error=%.1f head_correction=%d/%d",
                  commanded_pan, space_target.head_pitch, image_error,
                  head_corrections, kMaxHeadCorrections);
      if (!hardware_->MoveHead({commanded_pan, commanded_tilt})) {
        RCLCPP_WARN(node_->get_logger(),
                    "[FindSpace] fallo al mover cabeza para centrar contenedor; "
                    "se intentara continuar con geometria/base si aplica");
      }
      std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::duration<double>(space_target.sensor_settle_delay_sec)));
      continue;
    }

    if ((!image_centered || !pan_close) &&
        base_corrections < kMaxBaseCorrections) {
      const double pan_error = desired_pan - commanded_pan;
      double wz = 0.0;
      if (std::fabs(pan_error) > 0.03) {
        wz = std::clamp(pan_error, -kMaxWz, kMaxWz);
      } else if (pose.bbox_center_x > 0.0) {
        wz = image_error > 0.0 ? -kMaxWz : kMaxWz;
      }
      if (std::fabs(wz) > 0.001) {
        ++base_corrections;
        RCLCPP_INFO(node_->get_logger(),
                    "[FindSpace] precenter girando base para centrar vista "
                    "wz=%.3f image_error=%.1f desired_pan=%.3f "
                    "base_correction=%d/%d",
                    wz, image_error, desired_pan, base_corrections,
                    kMaxBaseCorrections);
        publishBasePulse(0.0, 0.0, wz, 550ms);
        commanded_pan = 0.0;
        commanded_tilt = space_target.head_pitch;
        hardware_->MoveHead({commanded_pan, commanded_tilt});
        std::this_thread::sleep_for(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(
                    space_target.sensor_settle_delay_sec)));
        continue;
      }
    }

    if (!in_ideal_zone && base_corrections < kMaxBaseCorrections) {
      double vx = 0.0;
      double vy = 0.0;
      double wz = 0.0;
      if (pose.x > kIdealXMax) {
        vx = kMaxVx;
      } else if (pose.x < kIdealXMin) {
        vx = -kMaxVx;
      }
      if (pose.y > kIdealAbsYMax) {
        vy = kMaxVy;
        if (std::fabs(pose.y) > 0.35) {
          wz = kMaxWz;
        }
      } else if (pose.y < -kIdealAbsYMax) {
        vy = -kMaxVy;
        if (std::fabs(pose.y) > 0.35) {
          wz = -kMaxWz;
        }
      }

      const double x_error = std::max(
          {0.0, pose.x - kIdealXMax, kIdealXMin - pose.x});
      const double y_error =
          std::max(0.0, std::fabs(pose.y) - kIdealAbsYMax);
      const auto pulse_ms = static_cast<int>(std::clamp(
          450.0 + 900.0 * std::max(x_error, y_error), 450.0, 950.0));

      ++base_corrections;
      RCLCPP_INFO(node_->get_logger(),
                  "[FindSpace] precenter corrigiendo base vx=%.3f vy=%.3f "
                  "wz=%.3f duracion=%.2fs base_correction=%d/%d",
                  vx, vy, wz, static_cast<double>(pulse_ms) / 1000.0,
                  base_corrections, kMaxBaseCorrections);
      publishBasePulse(vx, vy, wz, std::chrono::milliseconds(pulse_ms));
      commanded_pan = 0.0;
      commanded_tilt = space_target.head_pitch;
      hardware_->MoveHead({commanded_pan, commanded_tilt});
      std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::duration<double>(space_target.sensor_settle_delay_sec)));
      continue;
    }

    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] no se logro zona ideal tras correcciones; "
                "se continuara con geometria si sigue en zona escaneable");
    return true;
  }

  return true;
}

bool FindSpace::requestOwlContainerPose(OwlContainerPose &pose_out,
                                   std::string &error_out) {
  // OWLv2 se usa aqui como apoyo de precentrado. La decision final de espacio
  // seguro sigue en la geometria de los nodos *_drop_tf_node.
  if (!owl_v2_client_->wait_for_service(1s)) {
    error_out = "owl_service_not_available";
    return false;
  }

  auto request =
      std::make_shared<vision_interfaces::srv::GetDetections::Request>();
  request->queries = {"container_drop"};

  auto future = owl_v2_client_->async_send_request(request);
  if (future.wait_for(4s) != std::future_status::ready) {
    error_out = "owl_service_timeout";
    return false;
  }

  const auto response = future.get();
  if (!response || !response->success || response->detections.empty()) {
    error_out = response ? response->message : "empty_owl_response";
    if (error_out.empty()) {
      error_out = "no_owl_detections";
    }
    return false;
  }

  constexpr double kIdealXMin = 0.75;
  constexpr double kIdealXMax = 0.85;
  constexpr double kIdealAbsYMax = 0.22;
  constexpr double kScanXMin = 0.65;
  constexpr double kScanXMax = 1.60;
  constexpr double kScanAbsYMax = 0.65;
  constexpr double kImageCenterX = 320.0;

  bool found = false;
  double best_score = -1.0e9;
  std::string best_frame;
  std::string best_class;
  double best_confidence = 0.0;
  double best_bbox_center_x = 0.0;
  double best_bbox_width = 0.0;
  double best_bbox_height = 0.0;
  geometry_msgs::msg::TransformStamped best_tf;
  std::unordered_map<std::string, int> class_counts;

  for (const auto &det : response->detections) {
    const double width = std::max(0.0, det.x_max - det.x_min);
    const double height = std::max(0.0, det.y_max - det.y_min);
    if (width < 15.0 || height < 15.0) {
      continue;
    }

    const std::string frame_class = det.class_name;
    std::string sanitized = frame_class;
    std::replace(sanitized.begin(), sanitized.end(), ' ', '_');
    const int instance = class_counts[sanitized]++;
    const double area = width * height;
    const double center_x = 0.5 * (det.x_min + det.x_max);
    const std::string frame = owlFrameName(frame_class, instance);

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("base_link", frame, tf2::TimePointZero,
                                       500ms);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "[FindSpace] OWL candidato sin TF frame=%s error=%s",
                   frame.c_str(), ex.what());
      continue;
    }

    const double x = tf.transform.translation.x;
    const double y = tf.transform.translation.y;
    const bool in_scan_zone =
        x >= kScanXMin && x <= kScanXMax && std::fabs(y) <= kScanAbsYMax;
    const double ideal_x_error =
        std::max({0.0, kIdealXMin - x, x - kIdealXMax});
    const double ideal_y_error =
        std::max(0.0, std::fabs(y) - kIdealAbsYMax);
    const double scan_x_error =
        std::max({0.0, kScanXMin - x, x - kScanXMax});
    const double scan_y_error =
        std::max(0.0, std::fabs(y) - kScanAbsYMax);
    const double center_offset = std::fabs(center_x - kImageCenterX) /
                                 kImageCenterX;
    const double area_score = area / 10000.0;
    const double distance_score =
        (x >= kScanXMin && x <= kScanXMax)
            ? (kScanXMax - x) / std::max(0.001, kScanXMax - kScanXMin)
            : -std::fabs(x - kIdealXMin);
    const double score =
        0.35 * det.confidence + 1.20 * area_score +
        1.00 * distance_score + (in_scan_zone ? 3.0 : 0.0) -
        4.0 * std::hypot(ideal_x_error, ideal_y_error) -
        8.0 * std::hypot(scan_x_error, scan_y_error) -
        0.20 * center_offset;

    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] OWL candidato frame=%s conf=%.2f "
                "base=(%.3f %.3f %.3f) bbox_cx=%.1f area=%.0f "
                "score=%.3f in_scan=%s",
                frame.c_str(), det.confidence, x, y, tf.transform.translation.z,
                center_x, area, score, in_scan_zone ? "true" : "false");

    if (!found || score > best_score) {
      found = true;
      best_score = score;
      best_frame = frame;
      best_class = frame_class;
      best_confidence = det.confidence;
      best_bbox_center_x = center_x;
      best_bbox_width = width;
      best_bbox_height = height;
      best_tf = tf;
    }
  }

  if (!found) {
    error_out = "owl_detections_without_valid_tf_or_too_small";
    return false;
  }

  pose_out.detected = true;
  pose_out.class_name = best_class;
  pose_out.frame = best_frame;
  pose_out.confidence = best_confidence;
  pose_out.bbox_center_x = best_bbox_center_x;
  pose_out.bbox_width = best_bbox_width;
  pose_out.bbox_height = best_bbox_height;
  pose_out.x = best_tf.transform.translation.x;
  pose_out.y = best_tf.transform.translation.y;
  pose_out.z = best_tf.transform.translation.z;
  return true;
}
