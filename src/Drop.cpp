#include "executor_tools/Drop.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Drop::Drop(const rclcpp::Node::SharedPtr &node,
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
}

// =========================================================
// run
// =========================================================
Result Drop::run(const std::string &argument,
                 const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active()) {
    return {false, "FAILED", "goal_handle_inactive"};
  }

  // 1) Interpretamos el argumento del plan. En lugar de asumir que el
  // argumento es una TF, lo traducimos a un modo de percepcion:
  // mesa -> table_drop_start, contenedor -> container_drop_start.
  // estante/armario -> shelf_drop_start.
  const DropTarget drop_target = resolveDropTarget(argument);
  const std::string drop_frame = drop_target.frame;
  const std::string target_frame = "base_link";
  auto enable_client = enable_table_drop_tf_client_;
  if (drop_target.mode == "container") {
    enable_client = enable_container_drop_tf_client_;
  } else if (drop_target.mode == "shelf") {
    enable_client = enable_shelf_drop_tf_client_;
  }

  std::ostringstream ready_detail;
  ready_detail << "arg=" << (argument.empty() ? "<empty>" : argument)
               << " mode=" << drop_target.mode
               << " frame=" << drop_target.frame;
  publishFeedback(gh, DropState::READY, ready_detail.str());
  RCLCPP_INFO(node_->get_logger(),
              "[Drop] objetivo: argumento='%s' modo=%s TF '%s' respecto a '%s'",
              argument.c_str(), drop_target.mode.c_str(), drop_frame.c_str(),
              target_frame.c_str());

  publishFeedback(gh, DropState::PREPARE_SENSOR,
                  "deteniendo base, ajustando torso y cabeza");
  hardware_->MoveBase(0.0, 0.0, 0.0);

  // 2) Preparamos la postura de sensores segun el objetivo. Para mesa la
  // cabeza mira menos hacia abajo; para contenedores en piso mira mas abajo.
  if (!hardware_->MoveTorso(drop_target.torso_height)) {
    publishFeedback(gh, DropState::FAILED, "move_torso_failed");
    return {false, "FAILED", "move_torso_failed"};
  }

  if (!hardware_->MoveHead({0.0, drop_target.head_pitch})) {
    publishFeedback(gh, DropState::FAILED, "move_head_failed");
    return {false, "FAILED", "move_head_failed"};
  }

  const rclcpp::Time request_stamp = node_->now();
  // 3) Activamos solo el nodo de vision necesario. Cada nodo procesa una nube
  // y publica una TF estatica cuando encuentra un punto seguro.
  if (!setDropTfNodeEnabled(enable_client, drop_target.service_label, true)) {
    publishFeedback(gh, DropState::FAILED,
                    "enable_" + drop_target.mode + "_drop_tf_failed");
    return {false, "FAILED",
            "enable_" + drop_target.mode + "_drop_tf_failed"};
  }

  publishFeedback(gh, DropState::WAIT_DROP_TF,
                  target_frame + " <- " + drop_frame);

  const auto start_time = std::chrono::steady_clock::now();
  const auto max_wait = drop_target.mode == "container" ? 32s : 8s;
  geometry_msgs::msg::TransformStamped tf;
  bool found = false;
  std::string last_error;
  int attempts = 0;

  auto poseInBounds = [&drop_target](
                          const geometry_msgs::msg::TransformStamped &candidate) {
    const double x = candidate.transform.translation.x;
    const double y = candidate.transform.translation.y;
    const double z = candidate.transform.translation.z;
    const double distance_xy = std::hypot(x, y);
    const auto &b = drop_target.bounds;
    return x >= b.x_min && x <= b.x_max && std::fabs(y) <= b.abs_y_max &&
           z >= b.z_min && z <= b.z_max && distance_xy <= b.dist_xy_max;
  };

  // 4) Esperamos la TF fresca publicada por vision. Se ignoran TF viejas
  // anteriores a esta peticion y candidatos fuera de limites.
  while (gh->is_active() && std::chrono::steady_clock::now() - start_time < max_wait) {
    ++attempts;
    try {
      // Pedimos la transformacion mas reciente disponible. El nodo de vision
      // activo la publica como punto inicial seguro para soltar.
      tf = tf_buffer_->lookupTransform(target_frame, drop_frame,
                                       tf2::TimePointZero, 500ms);
      const rclcpp::Time tf_stamp(tf.header.stamp);
      if (tf_stamp.nanoseconds() != 0 && tf_stamp < request_stamp) {
        last_error = "stale_drop_tf stamp=" + std::to_string(tf_stamp.seconds()) +
                     " request_stamp=" +
                     std::to_string(request_stamp.seconds());
        RCLCPP_DEBUG(node_->get_logger(),
                     "[Drop] ignorando TF vieja %s <- %s intento=%d %s",
                     target_frame.c_str(), drop_frame.c_str(), attempts,
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
                     "[Drop] ignorando TF fuera de limites durante espera: %s",
                     last_error.c_str());
        std::this_thread::sleep_for(100ms);
        continue;
      }
      found = true;
      RCLCPP_INFO(node_->get_logger(),
                  "[Drop] TF encontrada tras %d intento(s)", attempts);
      break;
    } catch (const tf2::TransformException &ex) {
      last_error = ex.what();
      RCLCPP_DEBUG(node_->get_logger(),
                   "[Drop] esperando TF %s <- %s intento=%d error=%s",
                   target_frame.c_str(), drop_frame.c_str(), attempts,
                   last_error.c_str());
      std::this_thread::sleep_for(100ms);
    }
  }

  if (!gh->is_active()) {
    setDropTfNodeEnabled(enable_client, drop_target.service_label, false);
    publishFeedback(gh, DropState::FAILED, "goal_cancelled");
    return {false, "FAILED", "goal_cancelled"};
  }

  if (!found) {
    RCLCPP_WARN(node_->get_logger(),
                "[Drop] no se encontro TF despues de %d intento(s). ultimo_error=%s",
                attempts, last_error.c_str());
    publishFeedback(gh, DropState::FAILED,
                    "drop_tf_not_available: " + last_error);
    setDropTfNodeEnabled(enable_client, drop_target.service_label, false);
    return {false, "FAILED", "drop_tf_not_available"};
  }

  publishFeedback(gh, DropState::VALIDATE_DROP_TF);

  const double x = tf.transform.translation.x;
  const double y = tf.transform.translation.y;
  const double z = tf.transform.translation.z;
  const double distance_xy = std::hypot(x, y);

  RCLCPP_INFO(node_->get_logger(),
              "[Drop] TF %s en %s: posicion=(%.3f, %.3f, %.3f) "
              "dist_xy=%.3f quat=(%.3f, %.3f, %.3f, %.3f)",
              drop_frame.c_str(), target_frame.c_str(), x, y, z, distance_xy,
              tf.transform.rotation.x, tf.transform.rotation.y,
              tf.transform.rotation.z, tf.transform.rotation.w);

  const auto &bounds = drop_target.bounds;
  // 5) Validacion final en base_link. Los limites son distintos para mesa y
  // contenedor porque la altura esperada cambia mucho.
  if (x < bounds.x_min || x > bounds.x_max ||
      std::fabs(y) > bounds.abs_y_max || z < bounds.z_min ||
      z > bounds.z_max || distance_xy > bounds.dist_xy_max) {
    std::ostringstream oss;
    oss << "pose_out_of_bounds x=" << x << " y=" << y << " z=" << z
        << " dist_xy=" << distance_xy
        << " limites x=[" << bounds.x_min << "," << bounds.x_max
        << "] abs_y<=" << bounds.abs_y_max << " z=[" << bounds.z_min
        << "," << bounds.z_max << "] dist<=" << bounds.dist_xy_max
        << " mode=" << drop_target.mode;
    RCLCPP_WARN(node_->get_logger(), "[Drop] %s", oss.str().c_str());
    publishFeedback(gh, DropState::FAILED, oss.str());
    setDropTfNodeEnabled(enable_client, drop_target.service_label, false);
    return {false, "FAILED", oss.str()};
  }

  std::ostringstream ok_msg;
  ok_msg << "drop_start_tf_ready frame=" << drop_frame << " base=(" << x
         << ", " << y << ", " << z << ") mode=" << drop_target.mode;

  publishFeedback(gh, DropState::SUCCESS, ok_msg.str());
  setDropTfNodeEnabled(enable_client, drop_target.service_label, false);
  return {true, "SUCCESS", ok_msg.str()};
}

// =========================================================
// setDropTfNodeEnabled
// =========================================================
bool Drop::setDropTfNodeEnabled(
    const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr &client,
    const std::string &service_label, bool enabled) {
  if (!client->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[Drop] servicio %s no disponible", service_label.c_str());
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;

  auto future = client->async_send_request(request);
  if (future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[Drop] timeout al %s %s", enabled ? "activar" : "desactivar",
                 service_label.c_str());
    return false;
  }

  const auto response = future.get();
  RCLCPP_INFO(node_->get_logger(), "[Drop] %s %s: %s",
              service_label.c_str(), enabled ? "enable" : "disable",
              response->message.c_str());
  return response->success;
}

Drop::DropTarget Drop::resolveDropTarget(const std::string &argument) {
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

  DropTarget target;
  target.mode = "table";
  target.frame = "table_drop_start";
  target.service_label = "/vision/table_drop_tf/enable";
  target.torso_height = 0.3;
  target.head_pitch = -0.6;
  target.bounds = {0.15, 1.20, 0.80, 0.35, 1.40, 1.40};

  const bool explicit_container = contains("container_drop_start");
  const bool explicit_table = contains("table_drop_start");
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
    target.head_pitch = -0.85;
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

// =========================================================
// publishFeedback
// =========================================================
void Drop::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                           DropState state, const std::string &detail) {
  if (!gh->is_active()) {
    return;
  }

  std::string msg = "[Drop] " + stateName(state);
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
std::string Drop::stateName(DropState s) {
  switch (s) {
  case DropState::READY:
    return "READY";
  case DropState::PREPARE_SENSOR:
    return "PREPARE_SENSOR";
  case DropState::WAIT_DROP_TF:
    return "WAIT_DROP_TF";
  case DropState::VALIDATE_DROP_TF:
    return "VALIDATE_DROP_TF";
  case DropState::SUCCESS:
    return "SUCCESS";
  case DropState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}
