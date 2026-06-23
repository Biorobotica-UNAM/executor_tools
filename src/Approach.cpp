#include "executor_tools/Approach.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <thread>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "hardware_tools/Hardware.hpp"
#include "navigation_tools/Navigation.hpp" 

using namespace std::chrono_literals;

namespace {
constexpr double kReferenceConsistencyWarnM = 0.10;
constexpr double kObjectLateralSpeedMps = 0.05;
constexpr double kSafeDropFinalCorrectionToleranceM = 0.04;
constexpr double kSafeDropFinalCorrectionMaxM = 0.40;
constexpr double kSafeDropFinalCorrectionSpeedMps = 0.05;

std::string toLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool containsToken(const std::string &value, const std::string &token) {
    return toLowerCopy(value).find(token) != std::string::npos;
}

// Knowledge puede guardar objetos como "cutlery_1", "left_cutlery_1" o
// simplemente "cutlery"; normalizamos esas variantes antes de comparar.
std::string stripHandPrefix(const std::string &value) {
    if (value.rfind("left_", 0) == 0) {
        return value.substr(5);
    }
    if (value.rfind("right_", 0) == 0) {
        return value.substr(6);
    }
    return value;
}

std::string stripNumericSuffix(const std::string &value) {
    const auto sep = value.rfind('_');
    if (sep != std::string::npos && sep + 1 < value.size() &&
        std::all_of(value.begin() + sep + 1, value.end(),
                    [](unsigned char c) { return std::isdigit(c); })) {
        return value.substr(0, sep);
    }
    return value;
}

// Drop guarda en safe_drop_1 que objeto/mano motivaron el acercamiento.
// Approach usa este matcher para confirmar que esa mano sigue trayendo el
// mismo objeto antes de mover lateralmente la base.
bool objectTokenMatches(const std::string &expected, const std::string &held) {
    const std::string expected_norm = stripHandPrefix(toLowerCopy(expected));
    const std::string held_norm = stripHandPrefix(toLowerCopy(held));
    if (expected_norm.empty() || held_norm.empty() || held_norm == "nothing") {
        return false;
    }
    return expected_norm == held_norm ||
           expected_norm == stripNumericSuffix(held_norm) ||
           stripNumericSuffix(expected_norm) == held_norm;
}
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
    person_partial_distance_m_ =
        declare_or_get_double("approach.person_partial_distance_m", 2.5);
    person_partial_min_remaining_m_ =
        declare_or_get_double("approach.person_partial_min_remaining_m", 0.75);
    person_partial_timeout_s_ =
        declare_or_get_double("approach.person_partial_timeout_s", 45.0);
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
  
  std::size_t sep = argument.rfind('_');
  if (sep != std::string::npos &&
      sep + 1 < argument.size() &&
      std::all_of(argument.begin() + sep + 1, argument.end(),
                  [](unsigned char c) { return std::isdigit(c); })) {
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

  const bool is_person_target =
      (category == "customer" || category == "guest" ||
       category == "host" || category == "user");
  const bool is_safe_drop_target = category == "safe_drop";
  // safe_drop necesita terminar mucho mas cerca que un objeto/persona: Drop usa
  // esa marca como pose de manipulacion, no solo como destino de navegacion.
  const double kArrivalTolerance =
      is_person_target ? 1.8 : (is_safe_drop_target ? 0.08 : 0.30);
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
	              std::string final_correction_detail;
	              if (!applySafeDropFinalCorrection(category, global_pose, gh,
	                                                final_correction_detail)) {
	                  return {false, "FAILED", final_correction_detail};
	              }
	              if (!final_correction_detail.empty()) {
	                  detail += "; " + final_correction_detail;
	              }
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
              RCLCPP_WARN(
              node_->get_logger(),
              "[Approach] EXITING BY TF FALLBACK!"
              " distance=%.4f <= tolerance=%.4f — stopping nav.",
              distance_to_goal,
              kArrivalTolerance);

              // FIX: Go() publica en un tópico; el nav_controller sigue
              // activo cuando salimos por TF fallback. Publicamos la pose
              // actual como nuevo goal para que se detenga de inmediato.
              stopNavigation();

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
          stopNavigation();
          RCLCPP_ERROR(node_->get_logger(), "[Approach] Critical Error: Navigation timeout reached!");
          publishFeedback(gh, ApproachState::FAILED, "navigation_timeout");
          return {false, "FAILED", "navigation_timeout"};
      }

      std::this_thread::sleep_for(100ms);
  }

  // goal cancelado desde afuera
  {
      std::lock_guard<std::mutex> lock(navigation_status_mutex_);
      waiting_navigation_status_ = false;
  }
  stopNavigation();
  return {false, "FAILED", "goal_cancelled"};
}

Result Approach::runPartial(const std::string &argument,
                            const std::shared_ptr<GoalHandle> &gh)
{
  if (!gh || !gh->is_active())
    return {false, "FAILED", "goal_handle_inactive"};

  publishFeedback(gh, ApproachState::READY, "partial:" + argument);

  std::string category;
  std::string id;
  std::size_t sep = argument.find('_');
  if (sep != std::string::npos) {
      category = argument.substr(0, sep);
      id = argument.substr(sep + 1);
  } else {
      category = argument;
      id = "";
  }

  if (category != "customer" && category != "guest" &&
      category != "person" && category != "human") {
      publishFeedback(gh, ApproachState::FAILED, "partial_only_for_person_targets");
      return {false, "FAILED", "partial_only_for_person_targets"};
  }

  publishFeedback(gh, ApproachState::LOADING, argument);

  const nlohmann::json nav_array =
      knowledge_->get_world_state(category, id, "navigation");
  if (nav_array.empty() || !nav_array.is_array() || nav_array.size() < 7) {
      publishFeedback(gh, ApproachState::FAILED, "invalid_knowledge_data");
      return {false, "FAILED", "invalid_knowledge_data"};
  }

  geometry_msgs::msg::PoseStamped target_local_pose;
  target_local_pose.header.frame_id = "base_link";
  target_local_pose.header.stamp = node_->now();

  try {
      target_local_pose.pose.position.x = nav_array[0].get<double>();
      target_local_pose.pose.position.y = nav_array[1].get<double>();
      target_local_pose.pose.position.z = nav_array[2].get<double>();
      target_local_pose.pose.orientation.x = nav_array[3].get<double>();
      target_local_pose.pose.orientation.y = nav_array[4].get<double>();
      target_local_pose.pose.orientation.z = nav_array[5].get<double>();
      target_local_pose.pose.orientation.w = nav_array[6].get<double>();
  } catch (const std::exception &ex) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[Approach] Invalid partial navigation data for target %s_%s: %s",
                   category.c_str(), id.c_str(), ex.what());
      publishFeedback(gh, ApproachState::FAILED, "invalid_knowledge_data");
      return {false, "FAILED", "invalid_knowledge_data"};
  }

  const double target_distance =
      std::sqrt(target_local_pose.pose.position.x *
                    target_local_pose.pose.position.x +
                target_local_pose.pose.position.y *
                    target_local_pose.pose.position.y);

  if (target_distance <= person_partial_min_remaining_m_) {
      publishFeedback(gh, ApproachState::SUCCESS, "partial_goal_already_close");
      return {true, "SUCCESS", "partial_goal_already_close"};
  }

  const double partial_distance =
      std::min(person_partial_distance_m_,
               std::max(0.0, target_distance - person_partial_min_remaining_m_));

  if (partial_distance <= 0.0) {
      publishFeedback(gh, ApproachState::SUCCESS, "partial_goal_already_close");
      return {true, "SUCCESS", "partial_goal_already_close"};
  }

  geometry_msgs::msg::PoseStamped partial_local_pose = target_local_pose;
  const double ratio = partial_distance / target_distance;
  partial_local_pose.pose.position.x = target_local_pose.pose.position.x * ratio;
  partial_local_pose.pose.position.y = target_local_pose.pose.position.y * ratio;
  partial_local_pose.pose.position.z = 0.0;
  partial_local_pose.header.stamp = node_->now();

  geometry_msgs::msg::PoseStamped partial_global_pose;
  try {
      partial_global_pose = tf_buffer_->transform(partial_local_pose, "map", 200ms);
  } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[Approach] Partial TF translation crashed: %s",
                   ex.what());
      publishFeedback(gh, ApproachState::FAILED, "tf_transform_error");
      return {false, "FAILED", "tf_transform_error"};
  }

  RCLCPP_INFO(node_->get_logger(),
              "[Approach] Partial person approach for %s: target_dist=%.3f m partial=%.3f m remaining=%.3f m",
              argument.c_str(),
              target_distance,
              partial_distance,
              target_distance - partial_distance);
  publishFeedback(gh, ApproachState::APPROACHING, "partial_navigation_goal");

  resetNavigationStatus();
  navigation_->Go(partial_global_pose);

  const double kArrivalTolerance = 0.10;
  const auto start_nav_time = std::chrono::steady_clock::now();
  const auto max_duration =
      std::chrono::duration<double>(std::max(1.0, person_partial_timeout_s_));
  int last_reported_nav_status = -2;
  std::string last_reported_nav_text;

  while (gh->is_active()) {
      const auto nav_status = navigationStatusSnapshot();

      if (nav_status.has_status &&
          (nav_status.code != last_reported_nav_status ||
           nav_status.text != last_reported_nav_text)) {
          std::string detail = std::string("partial_navigation_status=") +
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
              std::lock_guard<std::mutex> lock(navigation_status_mutex_);
              waiting_navigation_status_ = false;
          }

          std::string detail = std::string("partial_navigation_status=") +
                               navigationStatusName(nav_status.code);
          if (!nav_status.text.empty()) {
              detail += ": " + nav_status.text;
          }

          if (nav_status.success) {
              publishFeedback(gh, ApproachState::SUCCESS, detail);
              return {true, "SUCCESS", detail};
          }

          publishFeedback(gh, ApproachState::FAILED, detail);
          return {false, "FAILED", detail};
      }

      try {
          const auto tf_robot =
              tf_buffer_->lookupTransform("map", "base_link",
                                          tf2::TimePointZero, 50ms);
          const double dx =
              partial_global_pose.pose.position.x - tf_robot.transform.translation.x;
          const double dy =
              partial_global_pose.pose.position.y - tf_robot.transform.translation.y;
          const double distance_to_goal = std::sqrt(dx * dx + dy * dy);

          if (distance_to_goal <= kArrivalTolerance) {
              {
                  std::lock_guard<std::mutex> lock(navigation_status_mutex_);
                  waiting_navigation_status_ = false;
              }
              RCLCPP_WARN(node_->get_logger(),
                          "[Approach] Partial EXITING BY TF FALLBACK!"
                          " distance=%.4f <= tolerance=%.4f — stopping nav.",
                          distance_to_goal,
                          kArrivalTolerance);

              // FIX: ídem al run() — el goal parcial sigue activo en el nav
              // stack. Publicamos la pose actual como nuevo goal para detenerlo.
              stopNavigation();

              publishFeedback(gh, ApproachState::SUCCESS,
                              "partial_arrived_by_tf_fallback");
              return {true, "SUCCESS", "partial_arrived_by_tf_fallback"};
          }
      } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                               "[Approach] Partial waiting for localization update: %s",
                               ex.what());
      }

      if (std::chrono::steady_clock::now() - start_nav_time > max_duration) {
          {
              std::lock_guard<std::mutex> lock(navigation_status_mutex_);
              waiting_navigation_status_ = false;
          }
          stopNavigation();
          publishFeedback(gh, ApproachState::FAILED, "partial_navigation_timeout");
          return {false, "FAILED", "partial_navigation_timeout"};
      }

      std::this_thread::sleep_for(100ms);
  }

  // goal cancelado desde afuera
  {
      std::lock_guard<std::mutex> lock(navigation_status_mutex_);
      waiting_navigation_status_ = false;
  }
  stopNavigation();
  return {false, "FAILED", "goal_cancelled"};
}

// =========================================================
// stopNavigation
// =========================================================
void Approach::stopNavigation()
{
    // Go() publica en un tópico simple (/nav_control/goal), no es un action,
    // así que no existe un Cancel() nativo. La estrategia es sobrescribir el
    // goal con la pose actual del robot: el nav_controller recibe "ve a donde
    // ya estás", distancia ≈ 0, y deja de comandar velocidades de inmediato.
    try {
        const auto tf_robot =
            tf_buffer_->lookupTransform("map", "base_link",
                                        tf2::TimePointZero, 50ms);
        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header.frame_id   = "map";
        current_pose.header.stamp      = node_->now();
        current_pose.pose.position.x   = tf_robot.transform.translation.x;
        current_pose.pose.position.y   = tf_robot.transform.translation.y;
        current_pose.pose.position.z   = tf_robot.transform.translation.z;
        current_pose.pose.orientation  = tf_robot.transform.rotation;
        navigation_->Go(current_pose);
        RCLCPP_INFO(node_->get_logger(),
                    "[Approach] stopNavigation: published current pose as goal"
                    " (x=%.3f y=%.3f) to halt nav_controller.",
                    current_pose.pose.position.x,
                    current_pose.pose.position.y);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(node_->get_logger(),
                    "[Approach] stopNavigation: TF unavailable, nav_controller"
                    " may keep running briefly: %s", ex.what());
    }
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

    const std::string right_value = right_object.get<std::string>();
    const std::string left_value = left_object.get<std::string>();
    const bool right_free = right_value == "nothing" || right_value.empty();
    const bool left_free = left_value == "nothing" || left_value.empty();
    const bool right_occupied = !right_free;
    const bool left_occupied = !left_free;

    const bool is_drop_target = category == "safe_drop";
    bool use_right_hand = false;
    bool drop_context_selected = false;
    const char *selection_reason = "free hand";

    if (is_drop_target) {
        // Para drop el ajuste lateral se alinea con la mano que trae el objeto;
        // es la contraparte de take, donde se prefiere la mano libre.
        selection_reason = "held object";
        if (!right_occupied && !left_occupied) {
            detail = "object_lateral_adjustment_skipped_no_held_object";
            RCLCPP_WARN(
                node_->get_logger(),
                "[Approach] Drop lateral adjustment skipped: no held object");
            publishFeedback(gh, ApproachState::APPROACHING, detail);
            return true;
        }

        const auto drop_hand =
            knowledge_->get_world_state(category, id, "drop_hand");
        const auto drop_object =
            knowledge_->get_world_state(category, id, "drop_object");
        // Este es el handoff Drop -> Approach: Drop escribe drop_hand/drop_object
        // justo antes de fallar por rango o planear, y Approach usa esos campos
        // para alinear con el brazo que realmente trae el objeto a soltar.
        if (drop_hand.is_string()) {
            const std::string hand = toLowerCopy(drop_hand.get<std::string>());
            const std::string object =
                drop_object.is_string() ? drop_object.get<std::string>() : "";
            const bool left_matches =
                hand == "left_hand" && left_occupied &&
                (object.empty() || objectTokenMatches(object, left_value));
            const bool right_matches =
                hand == "right_hand" && right_occupied &&
                (object.empty() || objectTokenMatches(object, right_value));

            if (left_matches || right_matches) {
                use_right_hand = right_matches;
                selection_reason = "drop requested object";
                drop_context_selected = true;
            } else if (!object.empty()) {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "[Approach] Drop lateral context ignored: hand=%s object=%s left=%s right=%s",
                    hand.c_str(), object.c_str(),
                    left_value.c_str(), right_value.c_str());
            }
        }

        if (drop_context_selected) {
            // Ya sabemos que brazo trae el objeto que Drop intento soltar.
        } else if (right_occupied && left_occupied) {
            const auto target_name =
                knowledge_->get_world_state(category, id, "name");
            const bool is_container_target =
                target_name.is_string() &&
                (containsToken(target_name.get<std::string>(), "container") ||
                 containsToken(target_name.get<std::string>(), "contenedor") ||
                 containsToken(target_name.get<std::string>(), "bote"));

            if (!is_container_target) {
                detail = "object_lateral_adjustment_skipped_ambiguous_held_hand";
                RCLCPP_WARN(
                    node_->get_logger(),
                    "[Approach] Drop lateral adjustment skipped: both hands are occupied");
                publishFeedback(gh, ApproachState::APPROACHING, detail);
                return true;
            }

            // Si las dos manos estan ocupadas y el destino es contenedor,
            // escogemos una mano fija para no bloquear el flujo por ambiguedad.
            use_right_hand = false;
            selection_reason = "container fallback held object";
        } else {
            use_right_hand = right_occupied;
        }
    } else {
        if (!right_free && !left_free) {
            detail = "object_lateral_adjustment_skipped_no_free_hand";
            RCLCPP_WARN(node_->get_logger(),
                        "[Approach] Object lateral adjustment skipped: no free hand");
            publishFeedback(gh, ApproachState::APPROACHING, detail);
            return true;
        }
        use_right_hand = right_free;
    }

    const double distance_m =
        use_right_hand ? object_lateral_offset_m_ : -object_lateral_offset_m_;
    const char *selected_hand = use_right_hand ? "right_hand" : "left_hand";
    const char *direction = use_right_hand ? "left" : "right";

    detail = std::string("object_lateral_adjustment_") + selected_hand +
             "_move_" + direction;

    RCLCPP_INFO(node_->get_logger(),
                "[Approach] Object centered at y=%.3f m. Using %s (%s), moving base %s %.3f m",
                target_y,
                selected_hand,
                selection_reason,
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

bool Approach::applySafeDropFinalCorrection(
    const std::string &category,
    const geometry_msgs::msg::PoseStamped &global_pose,
    const std::shared_ptr<GoalHandle> &gh,
    std::string &detail) {
    detail.clear();
    if (category != "safe_drop") {
        return true;
    }

    // La marca de navegacion puede quedarse corta por criterios del planner;
    // para safe_drop damos un ultimo pulso local antes del ajuste lateral.
    geometry_msgs::msg::PoseStamped target_in_base;
    try {
        auto map_pose_now = global_pose;
        map_pose_now.header.stamp = node_->now();
        target_in_base = tf_buffer_->transform(map_pose_now, "base_link", 200ms);
    } catch (const tf2::TransformException &ex) {
        detail = "safe_drop_final_correction_tf_failed";
        RCLCPP_WARN(node_->get_logger(),
                    "[Approach] Safe-drop final correction skipped: %s",
                    ex.what());
        publishFeedback(gh, ApproachState::APPROACHING, detail);
        return true;
    }

    const double dx = target_in_base.pose.position.x;
    const double dy = target_in_base.pose.position.y;
    const double residual = std::hypot(dx, dy);
    RCLCPP_INFO(node_->get_logger(),
                "[Approach] Safe-drop final residual after navigation: dx=%.3f dy=%.3f dist=%.3f",
                dx, dy, residual);

    if (residual <= kSafeDropFinalCorrectionToleranceM) {
        detail = "safe_drop_final_correction_not_needed";
        publishFeedback(gh, ApproachState::APPROACHING, detail);
        return true;
    }

    if (residual > kSafeDropFinalCorrectionMaxM) {
        detail = "safe_drop_final_correction_residual_too_large";
        RCLCPP_WARN(node_->get_logger(),
                    "[Approach] Safe-drop final residual %.3f m exceeds %.3f m",
                    residual, kSafeDropFinalCorrectionMaxM);
        publishFeedback(gh, ApproachState::FAILED, detail);
        return false;
    }

    detail = "safe_drop_final_correction";
    publishFeedback(gh, ApproachState::APPROACHING, detail);
    if (!moveBaseRelative(dx, dy, gh)) {
        detail = "safe_drop_final_correction_cancelled";
        publishFeedback(gh, ApproachState::FAILED, detail);
        return false;
    }

    detail = "safe_drop_final_correction_done";
    publishFeedback(gh, ApproachState::APPROACHING, detail);
    return true;
}

bool Approach::moveBaseRelative(
    double dx_m,
    double dy_m,
    const std::shared_ptr<GoalHandle> &gh) const {
    if (!hardware_) {
        return true;
    }

    const double distance = std::hypot(dx_m, dy_m);
    if (distance <= 0.0) {
        return true;
    }

    const double duration_s = distance / kSafeDropFinalCorrectionSpeedMps;
    const double vx = dx_m / duration_s;
    const double vy = dy_m / duration_s;
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

        hardware_->MoveBase(vx, vy, 0.0);
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
