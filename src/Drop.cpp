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

Drop::Drop(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Hardware> &hardware)
    : node_(node), hardware_(hardware) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

Result Drop::run(const std::string &argument,
                 const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active()) {
    return {false, "FAILED", "goal_handle_inactive"};
  }

  const DropTarget drop_target = resolveDropTarget(argument);
  const std::string target_frame = "base_link";

  std::ostringstream ready_detail;
  ready_detail << "arg=" << (argument.empty() ? "<empty>" : argument)
               << " mode=" << drop_target.mode
               << " frame=" << drop_target.frame;
  publishFeedback(gh, DropState::READY, ready_detail.str());

  // Esta accion ya no busca espacios seguros. FindSpace debe publicar primero
  // la TF objetivo; aqui solo validamos que exista antes de conectar brazo/mano.
  hardware_->MoveBase(0.0, 0.0, 0.0);
  publishFeedback(gh, DropState::WAIT_DROP_TF,
                  target_frame + " <- " + drop_target.frame);

  geometry_msgs::msg::TransformStamped tf;
  std::string last_error;
  bool found = false;
  int attempts = 0;
  const auto start_time = std::chrono::steady_clock::now();
  // DEBUG: si este bloque termina sin TF, no es un problema de manipulacion;
  // falta ejecutar find_space o el detector no dejo la marca esperada.
  while (gh->is_active() && std::chrono::steady_clock::now() - start_time < 3s) {
    ++attempts;
    try {
      tf = tf_buffer_->lookupTransform(target_frame, drop_target.frame,
                                       tf2::TimePointZero, 500ms);
      found = true;
      break;
    } catch (const tf2::TransformException &ex) {
      last_error = ex.what();
      std::this_thread::sleep_for(100ms);
    }
  }

  if (!gh->is_active()) {
    publishFeedback(gh, DropState::FAILED, "goal_cancelled");
    return {false, "FAILED", "goal_cancelled"};
  }

  if (!found) {
    RCLCPP_WARN(node_->get_logger(),
                "[Drop] TF %s no disponible despues de %d intento(s): %s",
                drop_target.frame.c_str(), attempts, last_error.c_str());
    publishFeedback(gh, DropState::FAILED,
                    "drop_tf_not_available_run_find_space_first");
    return {false, "FAILED", "drop_tf_not_available_run_find_space_first"};
  }

  publishFeedback(gh, DropState::VALIDATE_DROP_TF);

  const double x = tf.transform.translation.x;
  const double y = tf.transform.translation.y;
  const double z = tf.transform.translation.z;
  const double distance_xy = std::hypot(x, y);
  const auto &bounds = drop_target.bounds;

  RCLCPP_INFO(node_->get_logger(),
              "[Drop] TF %s en %s: posicion=(%.3f, %.3f, %.3f) "
              "dist_xy=%.3f quat=(%.3f, %.3f, %.3f, %.3f)",
              drop_target.frame.c_str(), target_frame.c_str(), x, y, z,
              distance_xy, tf.transform.rotation.x, tf.transform.rotation.y,
              tf.transform.rotation.z, tf.transform.rotation.w);

  if (x < bounds.x_min || x > bounds.x_max ||
      std::fabs(y) > bounds.abs_y_max || z < bounds.z_min ||
      z > bounds.z_max || distance_xy > bounds.dist_xy_max) {
    std::ostringstream oss;
    oss << "drop_tf_out_of_bounds x=" << x << " y=" << y << " z=" << z
        << " dist_xy=" << distance_xy
        << " limites x=[" << bounds.x_min << "," << bounds.x_max
        << "] abs_y<=" << bounds.abs_y_max << " z=[" << bounds.z_min
        << "," << bounds.z_max << "] dist<=" << bounds.dist_xy_max
        << " mode=" << drop_target.mode;
    RCLCPP_WARN(node_->get_logger(), "[Drop] %s", oss.str().c_str());
    publishFeedback(gh, DropState::FAILED, oss.str());
    return {false, "FAILED", oss.str()};
  }

  // TODO: cuando se implemente la manipulacion, usar drop_target.frame como
  // objetivo de mano, mover brazo al frame y abrir el gripper correspondiente.
  std::ostringstream ok_msg;
  ok_msg << "drop_tf_ready_pending_manipulation frame=" << drop_target.frame
         << " base=(" << x << ", " << y << ", " << z << ") mode="
         << drop_target.mode;
  publishFeedback(gh, DropState::SUCCESS, ok_msg.str());
  return {true, "SUCCESS", ok_msg.str()};
}

Drop::DropTarget Drop::resolveDropTarget(const std::string &argument) {
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
    target.bounds = {0.15, 1.35, 0.45, 0.08, 1.10, 1.50};
  }

  if (explicit_shelf || (looks_like_shelf && !looks_like_container)) {
    target.mode = "shelf";
    target.frame = "shelf_drop_start";
    target.bounds = {0.15, 1.35, 0.85, 0.30, 1.70, 1.55};
  }

  if (explicit_table) {
    target.mode = "table";
    target.frame = "table_drop_start";
    target.bounds = {0.15, 1.20, 0.80, 0.35, 1.40, 1.40};
  }

  return target;
}

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

std::string Drop::stateName(DropState s) {
  switch (s) {
  case DropState::READY:
    return "READY";
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
