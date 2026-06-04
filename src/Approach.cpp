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
// pfForceCallback
// =========================================================
void Approach::pfForceCallback(
    const geometry_msgs::msg::Vector3::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(force_mutex_);
  pf_force_x_ = msg->x;
  pf_force_y_ = msg->y;
}

// =========================================================
// moveRobot
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
  // GLOBAL EMPTY-MAP NAVIGATION
  // =====================================================
  if (is_person && gh->is_active()) {
    double initial_tx = 0.0;
    double initial_ty = 0.0;
    try {
      auto tf = tf_buffer_->lookupTransform("base_link", target_frame,
                                            tf2::TimePointZero, 100ms);
      initial_tx = tf.transform.translation.x;
      initial_ty = tf.transform.translation.y;

      double initial_dist =
          std::sqrt(initial_tx * initial_tx + initial_ty * initial_ty);
      if (initial_dist > 1.2) {
        RCLCPP_INFO(node_->get_logger(),
                    "Approach: Target is far (%.3f m). Starting global "
                    "empty-map navigation...",
                    initial_dist);

        // Temporarily reduce simple_move max_linear_speed for safe empty-map
        // navigation
        if (set_sm_params_client_->wait_for_service(
                std::chrono::milliseconds(500))) {
          auto req =
              std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
          rcl_interfaces::msg::Parameter param_max_speed;
          param_max_speed.name = "max_linear_speed";
          param_max_speed.value.type =
              rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          param_max_speed.value.double_value = 0.3;
          req->parameters.push_back(param_max_speed);
          set_sm_params_client_->async_send_request(req);
          RCLCPP_INFO(node_->get_logger(),
                      "Approach: Set simple_move max_linear_speed to 0.3 m/s");
        }

        // Temporarily increase potential_fields search window to stop further
        // from obstacles
        if (set_params_client_->wait_for_service(
                std::chrono::milliseconds(500))) {
          auto req =
              std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
          rcl_interfaces::msg::Parameter param_laser_x;
          param_laser_x.name = "laser_max_x";
          param_laser_x.value.type =
              rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          param_laser_x.value.double_value = 0.75;
          req->parameters.push_back(param_laser_x);

          rcl_interfaces::msg::Parameter param_cloud_x;
          param_cloud_x.name = "cloud_max_x";
          param_cloud_x.value.type =
              rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          param_cloud_x.value.double_value = 1.0;
          req->parameters.push_back(param_cloud_x);

          set_params_client_->async_send_request(req);
          RCLCPP_INFO(
              node_->get_logger(),
              "Approach: Set potential_fields laser_max_x to 0.95 m and "
              "cloud_max_x to 1.20 m");
        }

        bool global_goal_sent = false;
        auto last_goal_time =
            std::chrono::steady_clock::now() - 5s; // force immediate send

        while (gh->is_active()) {
          double tx = 0.0;
          double ty = 0.0;
          bool tf_ok = false;
          try {
            auto tf = tf_buffer_->lookupTransform("base_link", target_frame,
                                                  tf2::TimePointZero, 100ms);
            tx = tf.transform.translation.x;
            ty = tf.transform.translation.y;
            tf_ok = true;
          } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(
                node_->get_logger(),
                "Approach: TF lookup failed during global navigation: %s",
                ex.what());
          }

          if (tf_ok) {
            double dist = std::sqrt(tx * tx + ty * ty);
            if (dist <= 1.0) {
              RCLCPP_INFO(node_->get_logger(),
                          "Approach: Target reached proximity threshold (%.3f "
                          "m <= 1.0 m). Stopping global navigation.",
                          dist);
              auto stop_msg = std::make_shared<std_msgs::msg::Empty>();
              stop_nav_pub_->publish(*stop_msg);
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
                goal_msg.pose.position.x =
                    tf_map_target.transform.translation.x;
                goal_msg.pose.position.y =
                    tf_map_target.transform.translation.y;
                goal_msg.pose.position.z =
                    tf_map_target.transform.translation.z;
                goal_msg.pose.orientation = tf_map_target.transform.rotation;

                global_goal_pub_->publish(goal_msg);
                global_goal_sent = true;
                last_goal_time = now_time;

                RCLCPP_INFO(node_->get_logger(),
                            "Approach: Sent global goal for %s to [%.3f, %.3f] "
                            "in map frame",
                            target_frame.c_str(), goal_msg.pose.position.x,
                            goal_msg.pose.position.y);
              } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(node_->get_logger(),
                            "Approach: Map transform lookup failed: %s",
                            ex.what());
              }
            }
          }
          std::this_thread::sleep_for(100ms);
        }
      }
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "Approach: Initial TF lookup failed: %s",
                  ex.what());
    }
  }

  // =====================================================
  // APPROACHING LOOP
  // =====================================================
  publishFeedback(gh, ApproachState::APPROACHING);

  // Helper lambda to disable potential fields when we exit
  auto disable_pf = [this, is_person]() {
    if (is_person) {
      /*
      auto disable_msg = std::make_shared<std_msgs::msg::Bool>();
      disable_msg->data = false;
      pf_enable_pub_->publish(*disable_msg);
      RCLCPP_INFO(node_->get_logger(), "Approach: Disabled potential fields");
      */

      // Restore head position to look straight ahead
      hardware_->MoveHead({0.0, 0.0});
      RCLCPP_INFO(node_->get_logger(),
                  "Approach: Restored head position to (0.0, 0.0)");

      /*
      // Restore potential_fields parameters to their defaults
      if (set_params_client_->wait_for_service(
              std::chrono::milliseconds(500))) {
        auto req =
            std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        rcl_interfaces::msg::Parameter param_use_pot;
        param_use_pot.name = "use_pot_fields";
        param_use_pot.value.type =
            rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        param_use_pot.value.bool_value = false;
        req->parameters.push_back(param_use_pot);

        rcl_interfaces::msg::Parameter param_use_cloud;
        param_use_cloud.name = "use_point_cloud";
        param_use_cloud.value.type =
            rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        param_use_cloud.value.bool_value = false;
        req->parameters.push_back(param_use_cloud);

        rcl_interfaces::msg::Parameter param_laser_d0;
        param_laser_d0.name = "laser_pot_fields_d0";
        param_laser_d0.value.type =
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_laser_d0.value.double_value = 1.0;
        req->parameters.push_back(param_laser_d0);

        rcl_interfaces::msg::Parameter param_cloud_d0;
        param_cloud_d0.name = "cloud_pot_fields_d0";
        param_cloud_d0.value.type =
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_cloud_d0.value.double_value = 1.0;
        req->parameters.push_back(param_cloud_d0);

        set_params_client_->async_send_request(req);
        RCLCPP_INFO(node_->get_logger(),
                    "Approach: Restored potential_fields default parameters "
                    "(use_pot_fields=false, use_point_cloud=false, d0=1.0)");
      }
      */

      // Restore potential_fields search windows to launch defaults (0.55 m /
      // 0.80 m)
      if (set_params_client_->wait_for_service(
              std::chrono::milliseconds(500))) {
        auto req =
            std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter param_laser_x;
        param_laser_x.name = "laser_max_x";
        param_laser_x.value.type =
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_laser_x.value.double_value = 0.55;
        req->parameters.push_back(param_laser_x);

        rcl_interfaces::msg::Parameter param_cloud_x;
        param_cloud_x.name = "cloud_max_x";
        param_cloud_x.value.type =
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_cloud_x.value.double_value = 0.80;
        req->parameters.push_back(param_cloud_x);

        set_params_client_->async_send_request(req);
        RCLCPP_INFO(node_->get_logger(), "Approach: Restored potential_fields "
                                         "search windows (laser_max_x=0.55 m, "
                                         "cloud_max_x=0.80 m)");
      }

      // Restore simple_move parameter back to its default (0.8 m/s)
      if (set_sm_params_client_->wait_for_service(
              std::chrono::milliseconds(500))) {
        auto req =
            std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter param_max_speed;
        param_max_speed.name = "max_linear_speed";
        param_max_speed.value.type =
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_max_speed.value.double_value = 0.8;
        req->parameters.push_back(param_max_speed);
        set_sm_params_client_->async_send_request(req);
        RCLCPP_INFO(
            node_->get_logger(),
            "Approach: Restored simple_move max_linear_speed to 0.8 m/s");
      }
    }
  };

  // Enable potential fields if target is a person
  if (is_person) {
    // Lower head to see close obstacles in front of the robot
    hardware_->MoveHead({0.0, -0.4});
    RCLCPP_INFO(node_->get_logger(),
                "Approach: Lowered head to see close obstacles (-0.4 rad)");

    /*
    auto enable_msg = std::make_shared<std_msgs::msg::Bool>();
    enable_msg->data = true;
    pf_enable_pub_->publish(*enable_msg);
    RCLCPP_INFO(node_->get_logger(),
                "Approach: Enabled potential fields for person approach");

    // Dynamically set use_pot_fields and debug parameters to true
    if (set_params_client_->wait_for_service(std::chrono::milliseconds(500))) {
      auto req =
          std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

      rcl_interfaces::msg::Parameter param_use_pot;
      param_use_pot.name = "use_pot_fields";
      param_use_pot.value.type =
          rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      param_use_pot.value.bool_value = true;
      req->parameters.push_back(param_use_pot);

      rcl_interfaces::msg::Parameter param_debug;
      param_debug.name = "debug";
      param_debug.value.type =
          rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      param_debug.value.bool_value = true;
      req->parameters.push_back(param_debug);

      rcl_interfaces::msg::Parameter param_use_cloud;
      param_use_cloud.name = "use_point_cloud";
      param_use_cloud.value.type =
          rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      param_use_cloud.value.bool_value = true;
      req->parameters.push_back(param_use_cloud);

      rcl_interfaces::msg::Parameter param_laser_d0;
      param_laser_d0.name = "laser_pot_fields_d0";
      param_laser_d0.value.type =
          rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      param_laser_d0.value.double_value = 0.35;
      req->parameters.push_back(param_laser_d0);

      rcl_interfaces::msg::Parameter param_cloud_d0;
      param_cloud_d0.name = "cloud_pot_fields_d0";
      param_cloud_d0.value.type =
          rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      param_cloud_d0.value.double_value = 0.35;
      req->parameters.push_back(param_cloud_d0);

      set_params_client_->async_send_request(req);
      RCLCPP_INFO(
          node_->get_logger(),
          "Approach: Dynamically requested potential_fields parameters "
          "(use_pot_fields=true, debug=true, use_point_cloud=true, d0=0.35)");
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Approach: set_parameters service not available. Obstacle "
                  "avoidance might not activate if launched disabled.");
    }
    */
  }

  const double kTargetDist = 0.5; // 50 cm target distance
  const double kDistTol = 0.05;   // 5 cm distance tolerance
  const double kAngleTol = 0.1;   // ~5.7 degrees angular tolerance
  const double kYAlignTol = 0.03; // 3 cm sideways tolerance for objects

  // Proportional Gains
  const double kKpX = 0.4;
  const double kKpY = 0.4;
  const double kKpTheta = 0.8;

  // Safety Velocity Limits
  const double kMaxLinVel = 0.25; // m/s
  const double kMaxAngVel = 0.5;  // rad/s

  auto start_time = std::chrono::steady_clock::now();
  const auto kMaxDuration = 120s;

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

    // --- Person Target ---
    if (is_person) {
      double dist = std::sqrt(tx * tx + ty * ty);
      double angle = std::atan2(ty, tx);
      double dist_err = dist - kTargetDist;

      // Arrived check
      if (std::abs(dist_err) <= kDistTol && std::abs(angle) <= kAngleTol) {
        moveRobot(0.0, 0.0, 0.0);
        publishFeedback(gh, ApproachState::SUCCESS);
        disable_pf();
        return {true, "SUCCESS", "approached_person"};
      }

      // Retrieve latest repulsive force
      double rep_x = 0.0;
      double rep_y = 0.0;
      /*
      {
        std::lock_guard<std::mutex> lock(force_mutex_);
        rep_x = pf_force_x_;
        rep_y = pf_force_y_;
      }
      */

      // Compute attractive force components on base_link axes
      double ux = tx / dist;
      double uy = ty / dist;
      double att_mag = dist_err;
      if (att_mag > 0.3) {
        att_mag = 0.3;
      }
      double att_x = att_mag * ux;
      double att_y = att_mag * uy;

      // Compute net force vector (attraction + repulsion)
      double total_x = att_x + rep_x;
      double total_y = att_y + rep_y;

      // Calculate heading error to align with the net force vector
      double net_angle = std::atan2(total_y, total_x);

      RCLCPP_INFO(node_->get_logger(),
                  "Approach: dist_err=%.3f, att=[%.3f, %.3f], rep=[%.3f, "
                  "%.3f], net=[%.3f, %.3f], net_angle=%.3f",
                  dist_err, att_x, att_y, rep_x, rep_y, total_x, total_y,
                  net_angle);

      // Proportional control on net force
      double vx = 0.0;
      double vy = 0.0; // Enforce non-holonomic navigation (looking forward
                       // where the laser is)
      double wz = 0.0;

      // Always turn in place first to align with the net force vector
      // before moving forward
      if (std::abs(net_angle) <= 0.25) {
        vx = kKpX * total_x;
        wz = kKpTheta * net_angle;
      } else {
        vx = 0.0; // Turn in place to face the safe direction
        wz = kKpTheta * net_angle;
      }

      // Clamping velocities
      if (vx > kMaxLinVel)
        vx = kMaxLinVel;
      if (vx < 0.0)
        vx = 0.0;
      if (wz > kMaxAngVel)
        wz = kMaxAngVel;
      if (wz < -kMaxAngVel)
        wz = -kMaxAngVel;

      moveRobot(vx, vy, wz);
    } else {
      double y_err = ty;

      // Sideways aligned check
      if (std::abs(y_err) <= kYAlignTol) {
        moveRobot(0.0, 0.0, 0.0);
        publishFeedback(gh, ApproachState::SUCCESS);
        disable_pf();
        return {true, "SUCCESS", "aligned_object"};
      }

      double vy = kKpY * y_err;

      if (vy > kMaxLinVel)
        vy = kMaxLinVel;
      if (vy < -kMaxLinVel)
        vy = -kMaxLinVel;

      moveRobot(0.0, vy, 0.0);
    }

    // Timeout check
    if (std::chrono::steady_clock::now() - start_time > kMaxDuration) {
      moveRobot(0.0, 0.0, 0.0);
      publishFeedback(gh, ApproachState::FAILED, "timeout");
      disable_pf();
      return {false, "FAILED", "timeout"};
    }

    std::this_thread::sleep_for(50ms); // 20 Hz control loop
  }

  // Cancelled goal
  moveRobot(0.0, 0.0, 0.0);
  publishFeedback(gh, ApproachState::FAILED, "goal_cancelled");
  disable_pf();
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
