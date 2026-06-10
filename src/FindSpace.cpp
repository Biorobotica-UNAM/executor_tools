#include "executor_tools/FindSpace.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>
#include <unordered_map>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
FindSpace::FindSpace(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Hardware> &hardware)
    : node_(node), hardware_(hardware) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  enable_table_drop_tf_client_ = node_->create_client<std_srvs::srv::SetBool>(
      "/vision/table_drop_tf/enable");
  enable_container_drop_tf_client_ = node_->create_client<std_srvs::srv::SetBool>(
          "/vision/container_drop_tf/enable");
  enable_shelf_drop_tf_client_ = node_->create_client<std_srvs::srv::SetBool>(
      "/vision/shelf_drop_tf/enable");
  owl_v2_client_ = node_->create_client<vision_interfaces::srv::GetDetections>(
      "/vision/owl/owlv2_service");
}

// =========================================================
// run
// =========================================================
Result FindSpace::run(const std::string &argument,
                 const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active()) {
    return {false, "FAILED", "goal_handle_inactive"};
  }

  // 1) Interpretamos el argumento del plan. En lugar de asumir que el
  // argumento es una TF, lo traducimos a un modo de percepcion:
  // mesa -> table_drop_start, contenedor -> container_drop_start.
  // estante/armario -> shelf_drop_start.
  const SpaceTarget space_target = resolveDropTarget(argument);
  const std::string space_frame = space_target.frame;
  const std::string target_frame = "base_link";
  auto enable_client = enable_table_drop_tf_client_;
  if (space_target.mode == "container") {
    enable_client = enable_container_drop_tf_client_;
  } else if (space_target.mode == "shelf") {
    enable_client = enable_shelf_drop_tf_client_;
  }

  std::ostringstream ready_detail;
  ready_detail << "arg=" << (argument.empty() ? "<empty>" : argument)
               << " mode=" << space_target.mode
               << " frame=" << space_target.frame;
  publishFeedback(gh, FindSpaceState::READY, ready_detail.str());
  RCLCPP_INFO(node_->get_logger(),
              "[FindSpace] objetivo: argumento='%s' modo=%s TF '%s' respecto a '%s'",
              argument.c_str(), space_target.mode.c_str(), space_frame.c_str(),
              target_frame.c_str());

  publishFeedback(gh, FindSpaceState::PREPARE_SENSOR,
                  "deteniendo base, ajustando torso y cabeza");
  hardware_->MoveBase(0.0, 0.0, 0.0);

  // 2) Preparamos la postura de sensores segun el objetivo. Para mesa la
  // cabeza mira menos hacia abajo; para contenedores en piso mira mas abajo.
  if (!hardware_->MoveTorso(space_target.torso_height)) {
    publishFeedback(gh, FindSpaceState::FAILED, "move_torso_failed");
    return {false, "FAILED", "move_torso_failed"};
  }

  if (!hardware_->MoveHead({0.0, space_target.head_pitch})) {
    publishFeedback(gh, FindSpaceState::FAILED, "move_head_failed");
    return {false, "FAILED", "move_head_failed"};
  }

  if (space_target.sensor_settle_delay_sec > 0.0) {
    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] esperando %.2fs para estabilizar cabeza/camara antes "
                "del escaneo %s",
                space_target.sensor_settle_delay_sec,
                space_target.mode.c_str());
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(space_target.sensor_settle_delay_sec)));
  }

  if (space_target.mode == "container" &&
      !precenterContainerWithOwl(gh, space_target)) {
    // Si OWLv2 ve el contenedor pero esta demasiado lejos/lateral, avisamos
    // temprano. Ese mensaje sera la pista para que un nodo de navegacion
    // reaproxime al robot antes de volver a llamar find_space.
    publishFeedback(gh, FindSpaceState::FAILED, "container_out_of_scan_zone");
    hardware_->MoveBase(0.0, 0.0, 0.0);
    return {false, "FAILED", "container_out_of_scan_zone"};
  }

  const rclcpp::Time request_stamp = node_->now();
  // 3) Activamos solo el nodo de vision necesario. Cada nodo procesa una nube
  // y publica una TF fresca cuando encuentra un punto seguro.
  if (!setSpaceTfNodeEnabled(enable_client, space_target.service_label, true)) {
    publishFeedback(gh, FindSpaceState::FAILED,
                    "enable_" + space_target.mode + "_drop_tf_failed");
    return {false, "FAILED",
            "enable_" + space_target.mode + "_drop_tf_failed"};
  }

  publishFeedback(gh, FindSpaceState::WAIT_SPACE_TF,
                  target_frame + " <- " + space_frame);

  const auto start_time = std::chrono::steady_clock::now();
  const auto max_wait = space_target.mode == "container" ? 32s : 8s;
  geometry_msgs::msg::TransformStamped tf;
  bool found = false;
  std::string last_error;
  int attempts = 0;

  auto poseInBounds = [&space_target](
                          const geometry_msgs::msg::TransformStamped &candidate) {
    const double x = candidate.transform.translation.x;
    const double y = candidate.transform.translation.y;
    const double z = candidate.transform.translation.z;
    const double distance_xy = std::hypot(x, y);
    const auto &b = space_target.bounds;
    return x >= b.x_min && x <= b.x_max && std::fabs(y) <= b.abs_y_max &&
           z >= b.z_min && z <= b.z_max && distance_xy <= b.dist_xy_max;
  };

  // 4) Esperamos la TF fresca publicada por vision. Se ignoran TF viejas
  // anteriores a esta peticion y candidatos fuera de limites.
  // DEBUG: si aparece "ignorando TF vieja", el detector no genero una TF nueva
  // para este escaneo; revisar logs de container/table/shelf_drop_tf_node.
  while (gh->is_active() && std::chrono::steady_clock::now() - start_time < max_wait) {
    ++attempts;
    try {
      // Pedimos la transformacion mas reciente disponible. El nodo de vision
      // activo la publica como punto inicial seguro para soltar.
      tf = tf_buffer_->lookupTransform(target_frame, space_frame,
                                       tf2::TimePointZero, 500ms);
      const rclcpp::Time tf_stamp(tf.header.stamp);
      if (tf_stamp.nanoseconds() != 0 && tf_stamp < request_stamp) {
        last_error = "stale_drop_tf stamp=" + std::to_string(tf_stamp.seconds()) +
                     " request_stamp=" +
                     std::to_string(request_stamp.seconds());
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[FindSpace] ignorando TF vieja %s <- %s intento=%d %s",
                             target_frame.c_str(), space_frame.c_str(), attempts,
                             last_error.c_str());
        std::this_thread::sleep_for(100ms);
        continue;
      }
      if (!poseInBounds(tf)) {
        const double x = tf.transform.translation.x;
        const double y = tf.transform.translation.y;
        const double z = tf.transform.translation.z;
        const double distance_xy = std::hypot(x, y);
        last_error = "candidate_tf_out_of_bounds x=" + std::to_string(x) +
                     " y=" + std::to_string(y) +
                     " z=" + std::to_string(z) +
                     " dist_xy=" + std::to_string(distance_xy);
        RCLCPP_DEBUG(node_->get_logger(),
                     "[FindSpace] ignorando TF fuera de limites durante espera: %s",
                     last_error.c_str());
        std::this_thread::sleep_for(100ms);
        continue;
      }
      found = true;
      RCLCPP_INFO(node_->get_logger(),
                  "[FindSpace] TF encontrada tras %d intento(s) stamp=%.3f "
                  "request_stamp=%.3f",
                  attempts, tf_stamp.seconds(), request_stamp.seconds());
      break;
    } catch (const tf2::TransformException &ex) {
      last_error = ex.what();
      RCLCPP_DEBUG(node_->get_logger(),
                   "[FindSpace] esperando TF %s <- %s intento=%d error=%s",
                   target_frame.c_str(), space_frame.c_str(), attempts,
                   last_error.c_str());
      std::this_thread::sleep_for(100ms);
    }
  }

  if (!gh->is_active()) {
    setSpaceTfNodeEnabled(enable_client, space_target.service_label, false);
    publishFeedback(gh, FindSpaceState::FAILED, "goal_cancelled");
    return {false, "FAILED", "goal_cancelled"};
  }

  if (!found) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] no se encontro TF despues de %d intento(s). ultimo_error=%s",
                attempts, last_error.c_str());
    publishFeedback(gh, FindSpaceState::FAILED,
                    "find_space_tf_not_available: " + last_error);
    setSpaceTfNodeEnabled(enable_client, space_target.service_label, false);
    return {false, "FAILED", "find_space_tf_not_available"};
  }

  publishFeedback(gh, FindSpaceState::VALIDATE_SPACE_TF);

  const double x = tf.transform.translation.x;
  const double y = tf.transform.translation.y;
  const double z = tf.transform.translation.z;
  const double distance_xy = std::hypot(x, y);

  RCLCPP_INFO(node_->get_logger(),
              "[FindSpace] TF %s en %s: posicion=(%.3f, %.3f, %.3f) "
              "dist_xy=%.3f quat=(%.3f, %.3f, %.3f, %.3f)",
              space_frame.c_str(), target_frame.c_str(), x, y, z, distance_xy,
              tf.transform.rotation.x, tf.transform.rotation.y,
              tf.transform.rotation.z, tf.transform.rotation.w);

  const auto &bounds = space_target.bounds;
  // 5) Validacion final en base_link. Los limites son distintos para mesa y
  // contenedor porque la altura esperada cambia mucho.
  // DEBUG: si falla aqui, la TF existe pero no es una marca segura para usar
  // como objetivo de manipulacion; revisar la posicion base=(x,y,z) impresa.
  if (x < bounds.x_min || x > bounds.x_max ||
      std::fabs(y) > bounds.abs_y_max || z < bounds.z_min ||
      z > bounds.z_max || distance_xy > bounds.dist_xy_max) {
    std::ostringstream oss;
    oss << "pose_out_of_bounds x=" << x << " y=" << y << " z=" << z
        << " dist_xy=" << distance_xy
        << " limites x=[" << bounds.x_min << "," << bounds.x_max
        << "] abs_y<=" << bounds.abs_y_max << " z=[" << bounds.z_min
        << "," << bounds.z_max << "] dist<=" << bounds.dist_xy_max
        << " mode=" << space_target.mode;
    RCLCPP_WARN(node_->get_logger(), "[FindSpace] %s", oss.str().c_str());
    publishFeedback(gh, FindSpaceState::FAILED, oss.str());
    setSpaceTfNodeEnabled(enable_client, space_target.service_label, false);
    return {false, "FAILED", oss.str()};
  }

  std::ostringstream ok_msg;
  ok_msg << "find_space_tf_ready frame=" << space_frame << " base=(" << x
         << ", " << y << ", " << z << ") mode=" << space_target.mode;

  publishFeedback(gh, FindSpaceState::SUCCESS, ok_msg.str());
  setSpaceTfNodeEnabled(enable_client, space_target.service_label, false);
  return {true, "SUCCESS", ok_msg.str()};
}

// =========================================================
// setSpaceTfNodeEnabled
// =========================================================
bool FindSpace::setSpaceTfNodeEnabled(
    const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr &client,
    const std::string &service_label, bool enabled) {
  if (!client->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[FindSpace] servicio %s no disponible", service_label.c_str());
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;

  auto future = client->async_send_request(request);
  if (future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[FindSpace] timeout al %s %s", enabled ? "activar" : "desactivar",
                 service_label.c_str());
    return false;
  }

  const auto response = future.get();
  RCLCPP_INFO(node_->get_logger(), "[FindSpace] %s %s: %s",
              service_label.c_str(), enabled ? "enable" : "disable",
              response->message.c_str());
  return response->success;
}

bool FindSpace::precenterContainerWithOwl(
    const std::shared_ptr<GoalHandle> &gh,
    const SpaceTarget &space_target) {
  constexpr int kMaxHeadCorrections = 2;
  constexpr int kMaxBaseCorrections = 2;
  constexpr double kIdealXMin = 1.05;
  constexpr double kIdealXMax = 1.35;
  constexpr double kIdealAbsYMax = 0.22;
  constexpr double kScanXMin = 0.85;
  constexpr double kScanXMax = 1.60;
  constexpr double kScanAbsYMax = 0.65;
  constexpr double kImageCenterX = 320.0;
  constexpr double kImageCenterTolerancePx = 55.0;
  constexpr double kHeadPanMax = 0.55;
  constexpr double kHeadPanMin = -0.55;
  constexpr double kMaxVx = 0.10;
  constexpr double kMaxVy = 0.07;

  double commanded_pan = 0.0;
  int head_corrections = 0;
  int base_corrections = 0;
  // Esta etapa no decide el punto de drop. Solo intenta poner al robot/camara
  // en una postura donde el detector geometrico vea suficiente nube util.
  for (int attempt = 0; attempt <= kMaxHeadCorrections + kMaxBaseCorrections + 1;
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
      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] precenter OWL no disponible/sin deteccion: %s; "
                  "se usara fallback geometrico",
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
      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] contenedor fuera de la zona corregible actual: "
                  "base=(%.3f %.3f %.3f) scan_x=[%.2f %.2f] abs_y<=%.2f. "
                  "Se requiere reaproximacion externa.",
                  pose.x, pose.y, pose.z, kScanXMin, kScanXMax,
                  kScanAbsYMax);
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
      ++head_corrections;
      RCLCPP_INFO(node_->get_logger(),
                  "[FindSpace] precenter moviendo cabeza pan=%.3f tilt=%.3f "
                  "image_error=%.1f head_correction=%d/%d",
                  commanded_pan, space_target.head_pitch, image_error,
                  head_corrections, kMaxHeadCorrections);
      if (!hardware_->MoveHead({commanded_pan, space_target.head_pitch})) {
        RCLCPP_WARN(node_->get_logger(),
                    "[FindSpace] fallo al mover cabeza para centrar contenedor; "
                    "se intentara continuar con geometria/base si aplica");
      }
      std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::duration<double>(space_target.sensor_settle_delay_sec)));
      continue;
    }

    if (!in_ideal_zone && base_corrections < kMaxBaseCorrections) {
      double vx = 0.0;
      double vy = 0.0;
      if (pose.x > kIdealXMax) {
        vx = kMaxVx;
      } else if (pose.x < kIdealXMin) {
        vx = -kMaxVx;
      }
      if (pose.y > kIdealAbsYMax) {
        vy = kMaxVy;
      } else if (pose.y < -kIdealAbsYMax) {
        vy = -kMaxVy;
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
                  "duracion=%.2fs base_correction=%d/%d",
                  vx, vy, static_cast<double>(pulse_ms) / 1000.0,
                  base_corrections, kMaxBaseCorrections);
      publishBasePulse(vx, vy, 0.0, std::chrono::milliseconds(pulse_ms));
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

  bool found = false;
  double best_score = -1.0e9;
  std::string best_frame;
  std::string best_class;
  double best_confidence = 0.0;
  double best_bbox_center_x = 0.0;
  double best_bbox_width = 0.0;
  double best_bbox_height = 0.0;
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
    const double center_offset = std::fabs(center_x - 320.0) / 320.0;
    const double score = det.confidence + 0.00001 * area - 0.20 * center_offset;

    if (!found || score > best_score) {
      found = true;
      best_score = score;
      best_frame = owlFrameName(frame_class, instance);
      best_class = frame_class;
      best_confidence = det.confidence;
      best_bbox_center_x = center_x;
      best_bbox_width = width;
      best_bbox_height = height;
    }
  }

  if (!found) {
    error_out = "owl_detections_too_small";
    return false;
  }

  try {
    const auto tf = tf_buffer_->lookupTransform(
        "base_link", best_frame, tf2::TimePointZero, 500ms);
    pose_out.detected = true;
    pose_out.class_name = best_class;
    pose_out.frame = best_frame;
    pose_out.confidence = best_confidence;
    pose_out.bbox_center_x = best_bbox_center_x;
    pose_out.bbox_width = best_bbox_width;
    pose_out.bbox_height = best_bbox_height;
    pose_out.x = tf.transform.translation.x;
    pose_out.y = tf.transform.translation.y;
    pose_out.z = tf.transform.translation.z;
    return true;
  } catch (const tf2::TransformException &ex) {
    error_out = "owl_tf_not_available frame=" + best_frame +
                " error=" + ex.what();
    return false;
  }
}

void FindSpace::publishBasePulse(double vx, double vy, double wz,
                            std::chrono::milliseconds duration) {
  // Pulso corto y siempre con stop final. Esto evita que FindSpace se convierta
  // en navegacion completa: solo corrige errores pequenos antes del escaneo.
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < duration) {
    hardware_->MoveBase(vx, vy, wz);
    std::this_thread::sleep_for(50ms);
  }
  hardware_->MoveBase(0.0, 0.0, 0.0);
}

FindSpace::SpaceTarget FindSpace::resolveDropTarget(const std::string &argument) {
  // Mapeo conservador:
  // - left_table/table/mesa siguen usando la logica de mesa.
  // - trash/bin/box/container/bote/basura/caja usan la logica de contenedor.
  // - shelf/rack/cabinet/estante/armario/repisa usan la logica de estante.
  // - si no se reconoce nada, el default sigue siendo mesa para compatibilidad.
  std::string lower = argument;
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });

  const auto contains = [&lower](const std::string &needle) {
    return lower.find(needle) != std::string::npos;
  };

  SpaceTarget target;
  target.mode = "table";
  target.frame = "table_drop_start";
  target.service_label = "/vision/table_drop_tf/enable";
  target.torso_height = 0.3;
  target.head_pitch = -0.6;
  target.bounds = {0.15, 1.20, 0.80, 0.35, 1.40, 1.40};

  const bool explicit_container = contains("container_drop_start");
  const bool explicit_table = contains("table_drop_start");
  const bool explicit_shelf = contains("shelf_drop_start");
  const bool looks_like_container =
      contains("trash") || contains("bin") || contains("box") ||
      contains("container") || contains("bote") || contains("basura") ||
      contains("caja") || contains("contenedor") || contains("cubeta");
  const bool looks_like_table = contains("table") || contains("mesa");
  const bool looks_like_shelf =
      contains("shelf") || contains("rack") || contains("cabinet") ||
      contains("bookcase") || contains("estante") || contains("armario") ||
      contains("repisa");

  if (explicit_container || (looks_like_container && !looks_like_table)) {
    target.mode = "container";
    target.frame = "container_drop_start";
    target.service_label = "/vision/container_drop_tf/enable";
    target.torso_height = 0.2;
    target.head_pitch = -0.55;
    target.sensor_settle_delay_sec = 1.6;
    target.bounds = {0.15, 1.35, 0.45, 0.08, 1.10, 1.50};
  }

  if (explicit_shelf || (looks_like_shelf && !looks_like_container)) {
    target.mode = "shelf";
    target.frame = "shelf_drop_start";
    target.service_label = "/vision/shelf_drop_tf/enable";
    target.torso_height = 0.3;
    target.head_pitch = -0.45;
    target.bounds = {0.15, 1.35, 0.85, 0.30, 1.70, 1.55};
  }

  if (explicit_table) {
    target.mode = "table";
    target.frame = "table_drop_start";
    target.service_label = "/vision/table_drop_tf/enable";
    target.torso_height = 0.3;
    target.head_pitch = -0.6;
    target.bounds = {0.15, 1.20, 0.80, 0.35, 1.40, 1.40};
  }

  return target;
}

std::string FindSpace::owlFrameName(const std::string &class_name, int instance) {
  std::string sanitized = class_name;
  std::replace(sanitized.begin(), sanitized.end(), ' ', '_');
  return "OWL_" + sanitized + "_" + std::to_string(instance);
}

// =========================================================
// publishFeedback
// =========================================================
void FindSpace::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                           FindSpaceState state, const std::string &detail) {
  if (!gh->is_active()) {
    return;
  }

  std::string msg = "[FindSpace] " + stateName(state);
  if (!detail.empty()) {
    msg += ": " + detail;
  }

  auto fb = std::make_shared<ExecuteTask::Feedback>();
  fb->feedback.message = msg;
  gh->publish_feedback(fb);

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string FindSpace::stateName(FindSpaceState s) {
  switch (s) {
  case FindSpaceState::READY:
    return "READY";
  case FindSpaceState::PREPARE_SENSOR:
    return "PREPARE_SENSOR";
  case FindSpaceState::PRECENTER_CONTAINER_WITH_OWL:
    return "PRECENTER_CONTAINER_WITH_OWL";
  case FindSpaceState::WAIT_SPACE_TF:
    return "WAIT_SPACE_TF";
  case FindSpaceState::VALIDATE_SPACE_TF:
    return "VALIDATE_SPACE_TF";
  case FindSpaceState::SUCCESS:
    return "SUCCESS";
  case FindSpaceState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}
