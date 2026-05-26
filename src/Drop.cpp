#include "executor_tools/Drop.hpp"

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
  enable_drop_tf_client_ = node_->create_client<std_srvs::srv::SetBool>(
      "/vision/table_drop_tf/enable");
}

// =========================================================
// run
// =========================================================
Result Drop::run(const std::string &argument,
                 const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active()) {
    return {false, "FAILED", "goal_handle_inactive"};
  }

  // En esta primera fase Drop no mueve el brazo ni abre el gripper.
  // Su responsabilidad es preparar la percepcion y verificar que vision ya
  // publico una TF segura donde mas adelante iniciaremos la maniobra de soltar.
  const std::string drop_frame =
      argument.empty() ? "table_drop_start" : argument;
  const std::string target_frame = "base_link";

  publishFeedback(gh, DropState::READY, drop_frame);
  RCLCPP_INFO(node_->get_logger(),
              "[Drop] objetivo: validar TF '%s' respecto a '%s'",
              drop_frame.c_str(), target_frame.c_str());

  publishFeedback(gh, DropState::PREPARE_SENSOR,
                  "deteniendo base, ajustando torso y cabeza");
  hardware_->MoveBase(0.0, 0.0, 0.0);

  if (!hardware_->MoveTorso(0.3)) {
    publishFeedback(gh, DropState::FAILED, "move_torso_failed");
    return {false, "FAILED", "move_torso_failed"};
  }

  // Cabeza ligeramente hacia abajo para que el Kinect vea la superficie.
  if (!hardware_->MoveHead({0.0, -0.6})) {
    publishFeedback(gh, DropState::FAILED, "move_head_failed");
    return {false, "FAILED", "move_head_failed"};
  }

  const rclcpp::Time request_stamp = node_->now();
  if (!setDropTfNodeEnabled(true)) {
    publishFeedback(gh, DropState::FAILED, "enable_table_drop_tf_failed");
    return {false, "FAILED", "enable_table_drop_tf_failed"};
  }

  publishFeedback(gh, DropState::WAIT_DROP_TF,
                  target_frame + " <- " + drop_frame);

  const auto start_time = std::chrono::steady_clock::now();
  const auto max_wait = 8s;
  geometry_msgs::msg::TransformStamped tf;
  bool found = false;
  std::string last_error;
  int attempts = 0;

  auto poseInBounds = [](const geometry_msgs::msg::TransformStamped &candidate) {
    const double x = candidate.transform.translation.x;
    const double y = candidate.transform.translation.y;
    const double z = candidate.transform.translation.z;
    const double distance_xy = std::hypot(x, y);
    return x >= 0.15 && x <= 1.20 && std::fabs(y) <= 0.80 &&
           z >= 0.35 && z <= 1.40 && distance_xy <= 1.40;
  };

  while (gh->is_active() && std::chrono::steady_clock::now() - start_time < max_wait) {
    ++attempts;
    try {
      // Pedimos la transformacion mas reciente disponible. Si el nodo
      // table_drop_tf_node esta publicando correctamente, esta TF representa
      // el punto libre sobre la mesa, expresado aqui respecto a base_link.
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
    setDropTfNodeEnabled(false);
    publishFeedback(gh, DropState::FAILED, "goal_cancelled");
    return {false, "FAILED", "goal_cancelled"};
  }

  if (!found) {
    RCLCPP_WARN(node_->get_logger(),
                "[Drop] no se encontro TF despues de %d intento(s). ultimo_error=%s",
                attempts, last_error.c_str());
    publishFeedback(gh, DropState::FAILED,
                    "drop_tf_not_available: " + last_error);
    setDropTfNodeEnabled(false);
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

  // Validacion conservadora: esta TF solo debe pasar si esta frente al robot,
  // a una distancia razonable y con altura compatible con una mesa.
  if (x < 0.15 || x > 1.20 || std::fabs(y) > 0.80 || z < 0.35 || z > 1.40 ||
      distance_xy > 1.40) {
    std::ostringstream oss;
    oss << "pose_out_of_bounds x=" << x << " y=" << y << " z=" << z
        << " dist_xy=" << distance_xy
        << " limites x=[0.15,1.20] abs_y<=0.80 z=[0.35,1.40] dist<=1.40";
    RCLCPP_WARN(node_->get_logger(), "[Drop] %s", oss.str().c_str());
    publishFeedback(gh, DropState::FAILED, oss.str());
    setDropTfNodeEnabled(false);
    return {false, "FAILED", oss.str()};
  }

  std::ostringstream ok_msg;
  ok_msg << "drop_start_tf_ready frame=" << drop_frame << " base=(" << x
         << ", " << y << ", " << z << ")";

  publishFeedback(gh, DropState::SUCCESS, ok_msg.str());
  setDropTfNodeEnabled(false);
  return {true, "SUCCESS", ok_msg.str()};
}

// =========================================================
// setDropTfNodeEnabled
// =========================================================
bool Drop::setDropTfNodeEnabled(bool enabled) {
  if (!enable_drop_tf_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[Drop] servicio /vision/table_drop_tf/enable no disponible");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;

  auto future = enable_drop_tf_client_->async_send_request(request);
  if (future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[Drop] timeout al %s table_drop_tf_node",
                 enabled ? "activar" : "desactivar");
    return false;
  }

  const auto response = future.get();
  RCLCPP_INFO(node_->get_logger(), "[Drop] table_drop_tf_node %s: %s",
              enabled ? "enable" : "disable", response->message.c_str());
  return response->success;
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
