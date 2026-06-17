#include "executor_tools/Approach.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "navigation_tools/Navigation.hpp" 

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Approach::Approach(const rclcpp::Node::SharedPtr &node,
                   const std::shared_ptr<Hardware> &hardware,
                   const std::shared_ptr<Vision> &vision,
                   const std::shared_ptr<Knowledge> &knowledge,
                   const std::shared_ptr<Navigation> &navigation) 
    : node_(node), hardware_(hardware), vision_(vision), 
      knowledge_(knowledge), navigation_(navigation) // <── Asignación de la herramienta
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

  // ── STEP 1: PARSEO DINÁMICO DE CATEGORÍA E ID ──────────────────
  std::string category = "fruit";
  std::string id = "1";
  
  std::size_t sep = argument.find('_');
  if (sep != std::string::npos) {
      category = argument.substr(0, sep);
      id = argument.substr(sep + 1);
  } else {
      category = argument;
  }

  RCLCPP_INFO(node_->get_logger(), "[Approach] Querying Knowledge for key: %s_%s", 
              category.c_str(), id.c_str());

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

  local_pose.pose.position.x = nav_array[0].get<double>();
  local_pose.pose.position.y = nav_array[1].get<double>();
  local_pose.pose.position.z = nav_array[2].get<double>();
  
  local_pose.pose.orientation.x = nav_array[3].get<double>();
  local_pose.pose.orientation.y = nav_array[4].get<double>();
  local_pose.pose.orientation.z = nav_array[5].get<double>();
  local_pose.pose.orientation.w = nav_array[6].get<double>();

  RCLCPP_INFO(node_->get_logger(), 
              "[Approach] Local pose read successfully: Frontal(X): %.2fm, Lateral(Y): %.2fm", 
              local_pose.pose.position.x, local_pose.pose.position.y);

  // ── STEP 4: TRANSFORMACIÓN MATEMÁTICA A MAPA GLOBAL ───────────
  geometry_msgs::msg::PoseStamped global_pose;
  
  try {
      global_pose = tf_buffer_->transform(local_pose, "map", 200ms);

      RCLCPP_INFO(node_->get_logger(), "=================================================");
      RCLCPP_INFO(node_->get_logger(), "[Approach] TRANSFORMED SUCCESFULLY TO MAP FRAME!");
      RCLCPP_INFO(node_->get_logger(), "-> Global Target X (Map): %.3f m", global_pose.pose.position.x);
      RCLCPP_INFO(node_->get_logger(), "-> Global Target Y (Map): %.3f m", global_pose.pose.position.y);
      RCLCPP_INFO(node_->get_logger(), "=================================================");

  } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "[Approach] TF translation crashed: %s", ex.what());
      publishFeedback(gh, ApproachState::FAILED, "tf_transform_error");
      return {false, "FAILED", "tf_transform_error"};
  }

  // ── STEP 5: EJECUCIÓN ASÍNCRONA Y SEGUIMIENTO LOCAL ───────────
  publishFeedback(gh, ApproachState::APPROACHING, "Publishing navigation goal via tool...");
  
  navigation_->Go(global_pose);

  const double kArrivalTolerance = 0.05;
  auto start_nav_time = std::chrono::steady_clock::now();
  const auto kMaxNavDuration = 90s;

  RCLCPP_INFO(node_->get_logger(), "[Approach] Entering tracking loop. Monitoring distance to goal...");

  while (gh->is_active()) {
      try {
          auto tf_robot = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, 50ms);
          double current_rx = tf_robot.transform.translation.x;
          double current_ry = tf_robot.transform.translation.y;

          double dx = global_pose.pose.position.x - current_rx;
          double dy = global_pose.pose.position.y - current_ry;
          double distance_to_goal = std::sqrt(dx * dx + dy * dy);

          if (distance_to_goal <= kArrivalTolerance) {
              RCLCPP_INFO(node_->get_logger(), 
                          "[Approach] Goal reached successfully! Distance error: %.3f m", 
                          distance_to_goal);
              break;
          }
      } 
      catch (const tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                               "[Approach] Waiting for localization update: %s", ex.what());
      }

      if (std::chrono::steady_clock::now() - start_nav_time > kMaxNavDuration) {
          RCLCPP_ERROR(node_->get_logger(), "[Approach] Critical Error: Navigation timeout reached!");
          publishFeedback(gh, ApproachState::FAILED, "navigation_timeout");
          return {false, "FAILED", "navigation_timeout"};
      }

      std::this_thread::sleep_for(100ms); 
  }

  if (!gh->is_active()) {
      return {false, "FAILED", "goal_cancelled"};
  }

  publishFeedback(gh, ApproachState::SUCCESS, "arrived_successfully");
  return {true, "SUCCESS", "arrived_successfully"};
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