#include "executor_tools/Approach.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "hardware_tools/Hardware.hpp"
#include "navigation_tools/Navigation.hpp" 

using namespace std::chrono_literals;

namespace {
constexpr double kReferenceConsistencyWarnM = 0.10;
constexpr double kObjectLateralSpeedMps = 0.05;
}

// =========================================================
// Constructor
// =========================================================
Approach::Approach(const rclcpp::Node::SharedPtr &node,
                   const std::shared_ptr<Hardware> &hardware,
                   const std::shared_ptr<Vision> &vision,
                   const std::shared_ptr<Knowledge> &knowledge,
                   const std::shared_ptr<Navigation> &navigation) 
    : node_(node), hardware_(hardware), vision_(vision), 
      knowledge_(knowledge), navigation_(navigation)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    navigation_status_sub_ =
        node_->create_subscription<actionlib_msgs::msg::GoalStatus>(
            "/navigation/status",
            rclcpp::QoS(10).durability_volatile(),
            std::bind(&Approach::navigationStatusCallback,
                      this,
                      std::placeholders::_1));

    auto declare_or_get_double = [this](const std::string &name,
                                        double default_value) {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter<double>(name, default_value);
        }
        return node_->get_parameter(name).as_double();
    };

    object_lateral_offset_m_ =
        declare_or_get_double("approach.object_lateral_offset_m", 0.15);
    centered_y_threshold_m_ =
        declare_or_get_double("approach.centered_y_threshold_m", 0.08);
}

// =========================================================
// run (Extracción, Proyección y Navegación Bloqueante)
// =========================================================
Result Approach::run(const std::string &argument,
                     const std::shared_ptr<GoalHandle> &gh) 
{
  if (!gh || !gh->is_active())
    return {false, "FAILED", "goal_handle_inactive"};

  publishFeedback(gh, ApproachState::READY, argument);

  // ── STEP 1: PARSEO DINÁMICO DE CATEGORÍA E ID (100% Genérico) ──
  std::string category = "";
  std::string id = "";
  
  std::size_t sep = argument.find('_');
  if (sep != std::string::npos) {
      category = argument.substr(0, sep);
      id = argument.substr(sep + 1);
  } else {
      category = argument;
      id = ""; 
  }

  RCLCPP_INFO(node_->get_logger(), "[Approach] Querying Knowledge for key: %s%s%s", 
              category.c_str(), id.empty() ? "" : "_", id.c_str());

  // ── STEP 2: CONSULTA AL ESTADO DEL MUNDO ───────────────────────
  publishFeedback(gh, ApproachState::LOADING, argument);
  
  nlohmann::json nav_array = knowledge_->get_world_state(category, id, "navigation");

  if (nav_array.empty() || !nav_array.is_array() || nav_array.size() < 7) {
      RCLCPP_ERROR(node_->get_logger(), 
                   "[Approach] Empty or invalid navigation array for target: %s_%s", 
                   category.c_str(), id.c_str());
      publishFeedback(gh, ApproachState::FAILED, "invalid_knowledge_data");
      return {false, "FAILED", "invalid_knowledge_data"};
  }

  // ── STEP 3: CONSTRUCCIÓN DEL MENSAJE LOCAL (base_link) ────────
  geometry_msgs::msg::PoseStamped local_pose;
  local_pose.header.frame_id = "base_link";
  local_pose.header.stamp = node_->now();

  geometry_msgs::msg::PoseStamped global_pose;
  try {
      local_pose.pose.position.x = nav_array[0].get<double>();
      local_pose.pose.position.y = nav_array[1].get<double>();
      local_pose.pose.position.z = nav_array[2].get<double>();
      local_pose.pose.orientation.x = nav_array[3].get<double>();
      local_pose.pose.orientation.y = nav_array[4].get<double>();
      local_pose.pose.orientation.z = nav_array[5].get<double>();
      local_pose.pose.orientation.w = nav_array[6].get<double>();
  } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "[Approach] TF translation crashed: %s", ex.what());
      publishFeedback(gh, ApproachState::FAILED, "tf_transform_error");
      return {false, "FAILED", "tf_transform_error"};
  } catch (const std::exception &ex) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[Approach] Invalid navigation data for target %s_%s: %s",
                   category.c_str(), id.c_str(), ex.what());
      publishFeedback(gh, ApproachState::FAILED, "invalid_knowledge_data");
      return {false, "FAILED", "invalid_knowledge_data"};
  }

  RCLCPP_INFO(node_->get_logger(),
              "[Approach] Knowledge navigation interpreted in base_link: x=%.3f y=%.3f z=%.3f",
              local_pose.pose.position.x,
              local_pose.pose.position.y,
              local_pose.pose.position.z);

  // ── STEP 4: TRANSFORMACIÓN A MAPA PARA NAVEGACIÓN ─────────────
  try {
      global_pose = tf_buffer_->transform(local_pose, "map", 200ms);
  } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "[Approach] TF translation crashed: %s", ex.what());
      publishFeedback(gh, ApproachState::FAILED, "tf_transform_error");
      return {false, "FAILED", "tf_transform_error"};
  }

  try {
      auto map_pose_now = global_pose;
      map_pose_now.header.stamp = node_->now();
      const auto base_roundtrip =
          tf_buffer_->transform(map_pose_now, "base_link", 200ms);
      const double roundtrip_dx =
          base_roundtrip.pose.position.x - local_pose.pose.position.x;
      const double roundtrip_dy =
          base_roundtrip.pose.position.y - local_pose.pose.position.y;
      const double roundtrip_error =
          std::sqrt(roundtrip_dx * roundtrip_dx + roundtrip_dy * roundtrip_dy);

      if (roundtrip_error > kReferenceConsistencyWarnM) {
          RCLCPP_WARN(node_->get_logger(),
                      "[Approach] Reference mismatch at start: base->map->base error %.3f m",
                      roundtrip_error);
      } else {
          RCLCPP_INFO(node_->get_logger(),
                      "[Approach] Reference check OK at start: base->map->base error %.3f m",
                      roundtrip_error);
      }
  } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(),
                  "[Approach] Could not run initial base/map reference check: %s",
                  ex.what());
  }

  RCLCPP_INFO(node_->get_logger(), "=================================================");
  RCLCPP_INFO(node_->get_logger(), "[Approach] TARGET REFERENCES READY");
  RCLCPP_INFO(node_->get_logger(), "-> Base Target X/Y: %.3f / %.3f m",
              local_pose.pose.position.x,
              local_pose.pose.position.y);
  RCLCPP_INFO(node_->get_logger(), "-> Map Target X/Y:  %.3f / %.3f m",
              global_pose.pose.position.x,
              global_pose.pose.position.y);
  RCLCPP_INFO(node_->get_logger(), "=================================================");

  // ── STEP 5: EJECUCIÓN ASÍNCRONA Y FEEDBACK DE NAVEGACIÓN ─────
  publishFeedback(gh, ApproachState::APPROACHING, "Publishing navigation goal via tool...");

  RCLCPP_INFO(
    node_->get_logger(),
    "[Approach] Sending navigation goal:"
    "\n  frame_id = %s"
    "\n  stamp    = %.3f"
    "\n  x        = %.3f"
    "\n  y        = %.3f"
    "\n  z        = %.3f"
    "\n  qx qy qz qw = %.3f %.3f %.3f %.3f",
    global_pose.header.frame_id.c_str(),
    rclcpp::Time(global_pose.header.stamp).seconds(),
    global_pose.pose.position.x,
    global_pose.pose.position.y,
    global_pose.pose.position.z,
    global_pose.pose.orientation.x,
    global_pose.pose.orientation.y,
    global_pose.pose.orientation.z,
    global_pose.pose.orientation.w);

  resetNavigationStatus();
  navigation_->Go(global_pose);

  const double kArrivalTolerance = 0.05;
  auto start_nav_time = std::chrono::steady_clock::now();
  const auto kMaxNavDuration = 90s;
  int last_reported_nav_status = -2;
  std::string last_reported_nav_text;

  RCLCPP_INFO(node_->get_logger(),
              "[Approach] Entering tracking loop. Monitoring navigation status and distance to goal...");

  while (gh->is_active()) {
      const auto nav_status = navigationStatusSnapshot();

      if (nav_status.has_status &&
          (nav_status.code != last_reported_nav_status ||
           nav_status.text != last_reported_nav_text)) {
          std::string detail = std::string("navigation_status=") +
                               navigationStatusName(nav_status.code);
          if (!nav_status.text.empty()) {
              detail += ": " + nav_status.text;
          }
          publishFeedback(gh, ApproachState::APPROACHING, detail);
          last_reported_nav_status = nav_status.code;
          last_reported_nav_text = nav_status.text;
      }

      if (nav_status.done) {
          {
              RCLCPP_INFO(
              node_->get_logger(),
              "[Approach] nav_status.done=true"
              " code=%d (%s)"
              " success=%d"
              " text='%s'",
              nav_status.code,
              navigationStatusName(nav_status.code),
              nav_status.success,
              nav_status.text.c_str());
              std::lock_guard<std::mutex> lock(navigation_status_mutex_);
              waiting_navigation_status_ = false;
          }

          std::string detail = std::string("navigation_status=") +
                               navigationStatusName(nav_status.code);
          if (!nav_status.text.empty()) {
              detail += ": " + nav_status.text;
          }

          if (nav_status.success) {
              RCLCPP_INFO(node_->get_logger(),
                          "[Approach] Navigation reported success: %s",
                          detail.c_str());
              std::string lateral_detail;
              if (!applyObjectLateralAdjustment(category, id, local_pose, gh,
                                                lateral_detail)) {
                  return {false, "FAILED", lateral_detail};
              }
              if (!lateral_detail.empty()) {
                  detail += "; " + lateral_detail;
              }
              publishFeedback(gh, ApproachState::SUCCESS, detail);
              return {true, "SUCCESS", detail};
          }

          RCLCPP_ERROR(node_->get_logger(),
                       "[Approach] Navigation reported failure: %s",
                       detail.c_str());
          publishFeedback(gh, ApproachState::FAILED, detail);
          return {false, "FAILED", detail};
      }

      try {
          auto tf_robot = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, 50ms);
          double current_rx = tf_robot.transform.translation.x;
          double current_ry = tf_robot.transform.translation.y;

          double dx = global_pose.pose.position.x - current_rx;
          double dy = global_pose.pose.position.y - current_ry;
          double distance_to_goal = std::sqrt(dx * dx + dy * dy);

          auto map_pose_now = global_pose;
          map_pose_now.header.stamp = node_->now();
          const auto target_in_base =
              tf_buffer_->transform(map_pose_now, "base_link", 50ms);
          const double base_projected_distance =
              std::sqrt(target_in_base.pose.position.x *
                            target_in_base.pose.position.x +
                        target_in_base.pose.position.y *
                            target_in_base.pose.position.y);
          const double reference_distance_error =
              std::abs(distance_to_goal - base_projected_distance);

          RCLCPP_INFO_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 2000,
              "[Approach] Reference check: map_dist=%.3f m base_projected_dist=%.3f m diff=%.3f m",
              distance_to_goal, base_projected_distance,
              reference_distance_error);

          if (reference_distance_error > kReferenceConsistencyWarnM) {
              RCLCPP_WARN_THROTTLE(
                  node_->get_logger(), *node_->get_clock(), 2000,
                  "[Approach] Map/base reference mismatch while approaching: %.3f m",
                  reference_distance_error);
          }

          if (distance_to_goal <= kArrivalTolerance) {
              {
                  RCLCPP_INFO(
                  node_->get_logger(),
                  "[Approach] TF check:"
                  " distance=%.4f"
                  " tolerance=%.4f",
                  distance_to_goal,
                  kArrivalTolerance);
                  std::lock_guard<std::mutex> lock(navigation_status_mutex_);
                  waiting_navigation_status_ = false;
              }
              RCLCPP_ERROR(
              node_->get_logger(),
              "[Approach] EXITING BY TF FALLBACK!"
              " distance=%.4f"
              " tolerance=%.4f",
              distance_to_goal,
              kArrivalTolerance);
              std::string detail = "arrived_by_tf_fallback";
              std::string lateral_detail;
              if (!applyObjectLateralAdjustment(category, id, local_pose, gh,
                                                lateral_detail)) {
                  return {false, "FAILED", lateral_detail};
              }
              if (!lateral_detail.empty()) {
                  detail += "; " + lateral_detail;
              }
              publishFeedback(gh, ApproachState::SUCCESS, detail);
              return {true, "SUCCESS", detail};
          }
      } 
      catch (const tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                               "[Approach] Waiting for localization update: %s", ex.what());
      }

      if (std::chrono::steady_clock::now() - start_nav_time > kMaxNavDuration) {
          {
              std::lock_guard<std::mutex> lock(navigation_status_mutex_);
              waiting_navigation_status_ = false;
          }
          RCLCPP_ERROR(node_->get_logger(), "[Approach] Critical Error: Navigation timeout reached!");
          publishFeedback(gh, ApproachState::FAILED, "navigation_timeout");
          return {false, "FAILED", "navigation_timeout"};
      }

      std::this_thread::sleep_for(100ms);
  }

  if (!gh->is_active()) {
      std::lock_guard<std::mutex> lock(navigation_status_mutex_);
      waiting_navigation_status_ = false;
      return {false, "FAILED", "goal_cancelled"};
  }
  
  return {false, "FAILED", "goal_cancelled"};
}

// =========================================================
// Métodos Auxiliares
// =========================================================
void Approach::publishFeedback(const std::shared_ptr<GoalHandle>& gh, ApproachState state, const std::string& detail) {
    if (!gh || !gh->is_active()) return;
    std::string msg = "[approach] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;
    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);
    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

std::string Approach::stateName(ApproachState s) {
    switch (s) {
        case ApproachState::READY:       return "READY";
        case ApproachState::LOADING:     return "LOADING";
        case ApproachState::APPROACHING: return "APPROACHING";
        case ApproachState::SUCCESS:     return "SUCCESS";
        case ApproachState::FAILED:      return "FAILED";
        default:                         return "UNKNOWN";
    }
}

const char *Approach::navigationStatusName(int status) {
    switch (status) {
        case actionlib_msgs::msg::GoalStatus::PENDING:    return "PENDING";
        case actionlib_msgs::msg::GoalStatus::ACTIVE:     return "ACTIVE";
        case actionlib_msgs::msg::GoalStatus::PREEMPTED:  return "PREEMPTED";
        case actionlib_msgs::msg::GoalStatus::SUCCEEDED:  return "SUCCEEDED";
        case actionlib_msgs::msg::GoalStatus::ABORTED:    return "ABORTED";
        case actionlib_msgs::msg::GoalStatus::REJECTED:   return "REJECTED";
        case actionlib_msgs::msg::GoalStatus::PREEMPTING: return "PREEMPTING";
        case actionlib_msgs::msg::GoalStatus::RECALLING:  return "RECALLING";
        case actionlib_msgs::msg::GoalStatus::RECALLED:   return "RECALLED";
        case actionlib_msgs::msg::GoalStatus::LOST:       return "LOST";
        default:                                          return "UNKNOWN";
    }
}

bool Approach::isTerminalNavigationStatus(int status) {
    switch (status) {
        case actionlib_msgs::msg::GoalStatus::SUCCEEDED:
        case actionlib_msgs::msg::GoalStatus::PREEMPTED:
        case actionlib_msgs::msg::GoalStatus::ABORTED:
        case actionlib_msgs::msg::GoalStatus::REJECTED:
        case actionlib_msgs::msg::GoalStatus::RECALLED:
        case actionlib_msgs::msg::GoalStatus::LOST:
            return true;
        default:
            return false;
    }
}

bool Approach::isObjectTarget(const std::string &category,
                              const std::string &id) const {
    const auto manipulation =
        knowledge_->get_world_state(category, id, "manipulation");
    return manipulation.is_array() && manipulation.size() >= 7;
}

bool Approach::applyObjectLateralAdjustment(
    const std::string &category,
    const std::string &id,
    const geometry_msgs::msg::PoseStamped &local_pose,
    const std::shared_ptr<GoalHandle> &gh,
    std::string &detail) {
    detail.clear();

    if (!isObjectTarget(category, id) || object_lateral_offset_m_ <= 0.0) {
        return true;
    }

    const double target_y = local_pose.pose.position.y;
    if (std::abs(target_y) > centered_y_threshold_m_) {
        detail = "object_lateral_adjustment_skipped_not_centered";
        RCLCPP_INFO(node_->get_logger(),
                    "[Approach] Object lateral adjustment skipped: target y=%.3f m threshold=%.3f m",
                    target_y,
                    centered_y_threshold_m_);
        publishFeedback(gh, ApproachState::APPROACHING, detail);
        return true;
    }

    const auto right_object =
        knowledge_->get_world_state("robot", "", "right_hand");
    const auto left_object =
        knowledge_->get_world_state("robot", "", "left_hand");

    if (!right_object.is_string() || !left_object.is_string()) {
        detail = "object_lateral_adjustment_skipped_invalid_hand_state";
        RCLCPP_WARN(node_->get_logger(),
                    "[Approach] Object lateral adjustment skipped: invalid hand state");
        publishFeedback(gh, ApproachState::APPROACHING, detail);
        return true;
    }

    const bool right_free = right_object.get<std::string>() == "nothing";
    const bool left_free = left_object.get<std::string>() == "nothing";

    if (!right_free && !left_free) {
        detail = "object_lateral_adjustment_skipped_no_free_hand";
        RCLCPP_WARN(node_->get_logger(),
                    "[Approach] Object lateral adjustment skipped: no free hand");
        publishFeedback(gh, ApproachState::APPROACHING, detail);
        return true;
    }

    const double distance_m =
        right_free ? object_lateral_offset_m_ : -object_lateral_offset_m_;
    const char *selected_hand = right_free ? "right_hand" : "left_hand";
    const char *direction = right_free ? "left" : "right";

    detail = std::string("object_lateral_adjustment_") + selected_hand +
             "_move_" + direction;

    RCLCPP_INFO(node_->get_logger(),
                "[Approach] Object centered at y=%.3f m. Using %s, moving base %s %.3f m",
                target_y,
                selected_hand,
                direction,
                std::abs(distance_m));
    publishFeedback(gh, ApproachState::APPROACHING, detail);

    if (!moveBaseLaterally(distance_m, gh)) {
        detail = "object_lateral_adjustment_cancelled";
        publishFeedback(gh, ApproachState::FAILED, detail);
        return false;
    }

    detail += "_done";
    publishFeedback(gh, ApproachState::APPROACHING, detail);
    return true;
}

bool Approach::moveBaseLaterally(
    double distance_m,
    const std::shared_ptr<GoalHandle> &gh) const {
    if (!hardware_ || distance_m == 0.0) {
        return true;
    }

    const double direction = distance_m > 0.0 ? 1.0 : -1.0;
    const double duration_s = std::abs(distance_m) / kObjectLateralSpeedMps;
    const auto start_time = std::chrono::steady_clock::now();
    rclcpp::Rate rate(30.0);

    while (rclcpp::ok() &&
           std::chrono::duration<double>(
               std::chrono::steady_clock::now() - start_time).count() <
               duration_s) {
        if (!gh || !gh->is_active()) {
            hardware_->MoveBase(0.0, 0.0, 0.0);
            return false;
        }

        hardware_->MoveBase(0.0, direction * kObjectLateralSpeedMps, 0.0);
        rate.sleep();
    }

    hardware_->MoveBase(0.0, 0.0, 0.0);
    return true;
}

void Approach::resetNavigationStatus() {
    std::lock_guard<std::mutex> lock(navigation_status_mutex_);
    waiting_navigation_status_ = true;
    seen_navigation_progress_ = false;
    navigation_done_ = false;
    navigation_success_ = false;
    has_navigation_status_ = false;
    navigation_status_code_ = -1;
    navigation_status_text_.clear();
}

Approach::NavigationStatusSnapshot Approach::navigationStatusSnapshot() {
    std::lock_guard<std::mutex> lock(navigation_status_mutex_);
    return {
        has_navigation_status_,
        navigation_done_,
        navigation_success_,
        navigation_status_code_,
        navigation_status_text_};
}

void Approach::navigationStatusCallback(
    const actionlib_msgs::msg::GoalStatus::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(navigation_status_mutex_);

    // FIX: si ya completamos (done_=true), ignorar mensajes posteriores.
    // Sin esta guarda, el idle PENDING que publica el nav después de un
    // SUCCEEDED/ABORTED sobreescribe code_ y text_ antes de que el main
    // loop tome el snapshot, produciendo resultados con status incorrecto.
    if (!waiting_navigation_status_ || navigation_done_) {
        return;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "[Approach] navigationStatusCallback:"
        " status=%d (%s)"
        " waiting=%d"
        " seen_progress=%d"
        " done=%d"
        " success=%d"
        " text='%s'",
        msg->status,
        navigationStatusName(msg->status),
        waiting_navigation_status_,
        seen_navigation_progress_,
        navigation_done_,
        navigation_success_,
        msg->text.c_str());

    has_navigation_status_ = true;
    navigation_status_code_ = msg->status;
    navigation_status_text_ = msg->text;

    if (msg->status == actionlib_msgs::msg::GoalStatus::PENDING ||
        msg->status == actionlib_msgs::msg::GoalStatus::ACTIVE) {
        seen_navigation_progress_ = true;
    }

    if (seen_navigation_progress_ && isTerminalNavigationStatus(msg->status)) {
        navigation_done_ = true;
        navigation_success_ =
            (msg->status == actionlib_msgs::msg::GoalStatus::SUCCEEDED);
    }
}
