#include "executor_tools/Approach.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Approach::Approach(const rclcpp::Node::SharedPtr &node,
                   const std::shared_ptr<Hardware> &hardware,
                   const std::shared_ptr<Vision> &vision,
                   const std::shared_ptr<Knowledge> &knowledge) 
    : node_(node), hardware_(hardware), vision_(vision), knowledge_(knowledge) {

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pf_enable_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/navigation/navigation/potential_fields/enable", 10);

  pf_force_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
      "/navigation/navigation/potential_fields/pf_rejection_force",
      rclcpp::QoS(10).transient_local(),
      std::bind(&Approach::pfForceCallback, this, std::placeholders::_1));

  pf_cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/hardware/base/cmd_vel", rclcpp::SensorDataQoS());

  set_params_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(
      "/navigation/potential_fields/set_parameters");

  set_sm_params_client_ =
      node_->create_client<rcl_interfaces::srv::SetParameters>(
          "/navigation/simple_move/set_parameters");

  global_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/nav_control/goal", 10);

  stop_nav_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/navigation/navigation/stop", 10);
}

// =========================================================
// Callback de Fuerza de Campos Potenciales
// =========================================================
void Approach::pfForceCallback(
    const geometry_msgs::msg::Vector3::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(force_mutex_);
  pf_force_x_ = msg->x;
  pf_force_y_ = msg->y;
}

// =========================================================
// Envío de Comandos de Velocidad a la Base
// =========================================================
void Approach::moveRobot(double vx, double vy, double wz) {
  hardware_->MoveBase(vx, vy, wz);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = vx;
  cmd.linear.y = vy;
  cmd.angular.z = wz;
  pf_cmd_vel_pub_->publish(cmd);
}

// =========================================================
// Método de Ejecución Principal (FSM Reactiva)
// =========================================================
Result Approach::run(const std::string &argument,
                     const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active())
    return {false, "FAILED", "goal_handle_inactive"};

  publishFeedback(gh, ApproachState::READY, argument);

  // ── 1. Parseo Dinámico del Objetivo (Filtro Humanos vs Objetos) ──
  std::string target_frame = argument;
  bool is_person = false;

  if (argument == "person" || argument == "customer") {
    target_frame = "customer";
    is_person = true;
  } else if (argument == "host") {
    target_frame = "host";
    is_person = true;
  } else if (argument.rfind("guest_", 0) == 0) {
    target_frame = argument; // Captura guest_1, guest_2, etc.
    is_person = true;
  } else if (argument.find("OWL_") == std::string::npos && argument != "object") {
    target_frame = "OWL_" + argument + "_0";
  }

  RCLCPP_INFO(node_->get_logger(), "Approach starting: frame=%s (type=%s)",
              target_frame.c_str(), is_person ? "person" : "object");

  publishFeedback(gh, ApproachState::LOOKING_FOR_TF, target_frame);

  // ── 2. Escaneo Seguro de la TF en el Árbol de ROS 2 (Timeout 5s) ──
  bool tf_found = false;
  auto lookup_start = std::chrono::steady_clock::now();
  const auto kLookupTimeout = 5s;

  while (gh->is_active()) {
    try {
      tf_buffer_->lookupTransform("base_link", target_frame,
                                  tf2::TimePointZero, 100ms);
      tf_found = true;
      break;
    } catch (const tf2::TransformException &) {
      std::this_thread::sleep_for(100ms);
    }
    if (std::chrono::steady_clock::now() - lookup_start > kLookupTimeout)
      break;
  }

  if (!tf_found) {
    publishFeedback(gh, ApproachState::FAILED, "tf_not_found: " + target_frame);
    return {false, "FAILED", "tf_not_found"};
  }

  // ── 3. FASE A: Navegación de Larga Distancia (Solo Humanos) ──
  if (is_person && gh->is_active()) {
    try {
      auto tf = tf_buffer_->lookupTransform("base_link", target_frame,
                                            tf2::TimePointZero, 100ms);
      const double initial_tx = tf.transform.translation.x;
      const double initial_ty = tf.transform.translation.y;
      const double initial_dist =
          std::sqrt(initial_tx * initial_tx + initial_ty * initial_ty);

      if (initial_dist > 1.2) {
        RCLCPP_INFO(node_->get_logger(),
                    "Approach: target is far (%.3f m), starting global "
                    "empty-map navigation",
                    initial_dist);

        if (set_sm_params_client_->wait_for_service(std::chrono::milliseconds(500))) {
          auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
          rcl_interfaces::msg::Parameter param_max_speed;
          param_max_speed.name = "max_linear_speed";
          param_max_speed.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          param_max_speed.value.double_value = 0.3;
          req->parameters.push_back(param_max_speed);
          set_sm_params_client_->async_send_request(req);
          RCLCPP_INFO(node_->get_logger(), "Approach: set simple_move max_linear_speed to 0.3 m/s");
        }

        if (set_params_client_->wait_for_service(std::chrono::milliseconds(500))) {
          auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
          rcl_interfaces::msg::Parameter param_laser_x;
          param_laser_x.name = "laser_max_x";
          param_laser_x.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          param_laser_x.value.double_value = 0.75;
          req->parameters.push_back(param_laser_x);

          rcl_interfaces::msg::Parameter param_cloud_x;
          param_cloud_x.name = "cloud_max_x";
          param_cloud_x.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          param_cloud_x.value.double_value = 1.0;
          req->parameters.push_back(param_cloud_x);

          set_params_client_->async_send_request(req);
          RCLCPP_INFO(node_->get_logger(), "Approach: set potential_fields search window");
        }

        auto last_goal_time = std::chrono::steady_clock::now() - 5s;

        while (gh->is_active()) {
          double tx = 0.0;
          double ty = 0.0;
          bool tf_ok = false;

          try {
            auto current_tf = tf_buffer_->lookupTransform(
                "base_link", target_frame, tf2::TimePointZero, 100ms);
            tx = current_tf.transform.translation.x;
            ty = current_tf.transform.translation.y;
            tf_ok = true;
          } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(node_->get_logger(),
                        "Approach: TF lookup failed during global navigation: %s",
                        ex.what());
          }

          if (tf_ok) {
            const double dist = std::sqrt(tx * tx + ty * ty);
            if (dist <= 1.0) {
              RCLCPP_INFO(node_->get_logger(),
                          "Approach: target reached proximity threshold "
                          "(%.3f m <= 1.0 m), stopping global navigation",
                          dist);
              std_msgs::msg::Empty stop_msg;
              stop_nav_pub_->publish(stop_msg);
              break;
            }

            auto now_time = std::chrono::steady_clock::now();
            if (now_time - last_goal_time > 3s) {
              try {
                auto tf_map_target = tf_buffer_->lookupTransform(
                    "map", target_frame, tf2::TimePointZero, 100ms);

                geometry_msgs::msg::PoseStamped goal_msg;
                goal_msg.header.frame_id = "map";
                goal_msg.header.stamp = node_->now();
                goal_msg.pose.position.x = tf_map_target.transform.translation.x;
                goal_msg.pose.position.y = tf_map_target.transform.translation.y;
                goal_msg.pose.position.z = tf_map_target.transform.translation.z;
                goal_msg.pose.orientation = tf_map_target.transform.rotation;

                global_goal_pub_->publish(goal_msg);
                last_goal_time = now_time;

                RCLCPP_INFO(node_->get_logger(),
                            "Approach: sent global goal for %s to [%.3f, %.3f]",
                            target_frame.c_str(), goal_msg.pose.position.x,
                            goal_msg.pose.position.y);
              } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(node_->get_logger(), "Approach: map transform lookup failed: %s", ex.what());
              }
            }
          }
          std::this_thread::sleep_for(100ms);
        }
      }
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "Approach: initial TF lookup failed: %s", ex.what());
    }
  }

  // ── 4. FASE B: Aproximación Reactiva de Corta Distancia ──
  publishFeedback(gh, ApproachState::APPROACHING);

  auto restore_after_person = [this, is_person]() {
    if (!is_person)
      return;

    hardware_->MoveHead({0.0, 0.0});
    RCLCPP_INFO(node_->get_logger(), "Approach: restored head position to (0.0, 0.0)");

    if (set_params_client_->wait_for_service(std::chrono::milliseconds(500))) {
      auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      rcl_interfaces::msg::Parameter param_laser_x;
      param_laser_x.name = "laser_max_x";
      param_laser_x.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      param_laser_x.value.double_value = 0.55;
      req->parameters.push_back(param_laser_x);

      rcl_interfaces::msg::Parameter param_cloud_x;
      param_cloud_x.name = "cloud_max_x";
      param_cloud_x.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      param_cloud_x.value.double_value = 0.80;
      req->parameters.push_back(param_cloud_x);

      set_params_client_->async_send_request(req);
      RCLCPP_INFO(node_->get_logger(), "Approach: restored potential_fields search windows");
    }

    if (set_sm_params_client_->wait_for_service(std::chrono::milliseconds(500))) {
      auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      rcl_interfaces::msg::Parameter param_max_speed;
      param_max_speed.name = "max_linear_speed";
      param_max_speed.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      param_max_speed.value.double_value = 0.8;
      req->parameters.push_back(param_max_speed);
      set_sm_params_client_->async_send_request(req);
      RCLCPP_INFO(node_->get_logger(), "Approach: restored simple_move max_linear_speed to 0.8 m/s");
    }
  };

  if (is_person) {
    hardware_->MoveHead({0.0, -0.4});
    RCLCPP_INFO(node_->get_logger(), "Approach: lowered head to see close obstacles (-0.4 rad)");
  }

  const double kTargetDist = 0.5;
  const double kDistTol = 0.05;
  const double kAngleTol = 0.1;
  const double kYAlignTol = 0.03;

  const double kKpX = 0.4;
  const double kKpY = 0.4;
  const double kKpTheta = 0.8;

  const double kMaxLinVel = 0.25;
  const double kMaxAngVel = 0.5;

  auto start_time = std::chrono::steady_clock::now();
  const auto kMaxDuration = 120s;

  bool checked_hands = false;
  double y_offset = 0.0;
  const double kTargetDistObj = 0.55;

  // Ciclo reactivo de alta frecuencia (20 Hz)
  while (gh->is_active()) {
    double tx = 0.0;
    double ty = 0.0;

    try {
      auto tf = tf_buffer_->lookupTransform("base_link", target_frame,
                                            tf2::TimePointZero, 100ms);
      tx = tf.transform.translation.x;
      ty = tf.transform.translation.y;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "Approach: TF lookup failed: %s", ex.what());
      std::this_thread::sleep_for(50ms);
      continue;
    }

    if (is_person) {
      const double dist = std::sqrt(tx * tx + ty * ty);
      const double angle = std::atan2(ty, tx);
      const double dist_err = dist - kTargetDist;

      if (std::abs(dist_err) <= kDistTol && std::abs(angle) <= kAngleTol) {
        moveRobot(0.0, 0.0, 0.0);
        publishFeedback(gh, ApproachState::SUCCESS);
        restore_after_person();
        return {true, "SUCCESS", "approached_person"};
      }

      double rep_x = 0.0;
      double rep_y = 0.0;

      // Nota: Descomentar este bloque si deseas activar el lazo con potential_fields
      /*
      {
        std::lock_guard<std::mutex> lock(force_mutex_);
        rep_x = pf_force_x_;
        rep_y = pf_force_y_;
      }
      */

      const double ux = tx / dist;
      const double uy = ty / dist;
      const double att_mag = std::min(dist_err, 0.3);
      const double att_x = att_mag * ux;
      const double att_y = att_mag * uy;

      const double total_x = att_x + rep_x;
      const double total_y = att_y + rep_y;
      const double net_angle = std::atan2(total_y, total_x);

      RCLCPP_INFO(node_->get_logger(),
                  "Approach: dist_err=%.3f, att=[%.3f, %.3f], rep=[%.3f, %.3f], net=[%.3f, %.3f], net_angle=%.3f",
                  dist_err, att_x, att_y, rep_x, rep_y, total_x, total_y, net_angle);

      double vx = 0.0;
      double vy = 0.0;
      double wz = 0.0;

      if (std::abs(net_angle) <= 0.25) {
        vx = kKpX * total_x;
        wz = kKpTheta * net_angle;
      } else {
        wz = kKpTheta * net_angle;
      }

      vx = std::clamp(vx, 0.0, kMaxLinVel);
      wz = std::clamp(wz, -kMaxAngVel, kMaxAngVel);

      moveRobot(vx, vy, wz);
    } else {
      // =================================================================
      // RAMA DE OBJETO CONFIGURABLE SEGÚN MANO LIBRE
      // =================================================================
      const double x_err = tx - kTargetDistObj;

      if (!checked_hands && tx <= 0.75) {
        RCLCPP_INFO(node_->get_logger(), "[Approach] Close to object (%.2fm). Verifying hand vacancy...", tx);

        nlohmann::json left_object  = knowledge_->get_world_state("robot", "", "left_hand");
        nlohmann::json right_object = knowledge_->get_world_state("robot", "", "right_hand");

        std::string left_val  = left_object.is_string() ? left_object.get<std::string>() : "";
        std::string right_val = right_object.is_string() ? right_object.get<std::string>() : "";

        if (right_val == "empty" || right_val.empty()) {
            RCLCPP_INFO(node_->get_logger(), "[Approach] Right hand is empty! Shifting robot 15cm LEFT.");
            y_offset = -0.15; 
        }
        else if (left_val == "empty" || left_val.empty()) {
            RCLCPP_INFO(node_->get_logger(), "[Approach] Left hand is empty! Shifting robot 15cm RIGHT.");
            y_offset = 0.15;
        }
        else {
            RCLCPP_WARN(node_->get_logger(), "[Approach] Both hands are busy or data malformed! Staying centered.");
            y_offset = 0.0;
        }
        checked_hands = true;
      }

      const double y_err = ty - y_offset;

      if (std::abs(x_err) <= kDistTol && std::abs(y_err) <= kYAlignTol) {
        moveRobot(0.0, 0.0, 0.0);
        publishFeedback(gh, ApproachState::SUCCESS);
        restore_after_person();
        return {true, "SUCCESS", "aligned_object_with_arm_offset"};
      }

      double vx = std::clamp(kKpX * x_err, -kMaxLinVel, kMaxLinVel);
      double vy = std::clamp(kKpY * y_err, -kMaxLinVel, kMaxLinVel);

      if (vx < 0.0) vx = 0.0; 

      moveRobot(vx, vy, 0.0); 
    }

    if (std::chrono::steady_clock::now() - start_time > kMaxDuration) {
      moveRobot(0.0, 0.0, 0.0);
      publishFeedback(gh, ApproachState::FAILED, "timeout");
      restore_after_person();
      return {false, "FAILED", "timeout"};
    }

    std::this_thread::sleep_for(50ms);
  }

  moveRobot(0.0, 0.0, 0.0);
  publishFeedback(gh, ApproachState::FAILED, "goal_cancelled");
  restore_after_person();
  return {false, "FAILED", "goal_cancelled"};
}

// =========================================================
// Publicación de Feedback de la Acción
// =========================================================
void Approach::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                               ApproachState state, const std::string &detail) {
  if (!gh || !gh->is_active())
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
// Mapeo Textual de Estados de la FSM
// =========================================================
std::string Approach::stateName(ApproachState s) {
  switch (s) {
    case ApproachState::READY:           return "READY";
    case ApproachState::LOOKING_FOR_TF:  return "LOOKING_FOR_TF";
    case ApproachState::APPROACHING:     return "APPROACHING";
    case ApproachState::STABILIZING:     return "STABILIZING";
    case ApproachState::SUCCESS:         return "SUCCESS";
    case ApproachState::FAILED:          return "FAILED";
    default:                             return "UNKNOWN";
  }
}