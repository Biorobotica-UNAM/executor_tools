#include "executor_tools/Approach.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

// Incluimos la interfaz nativa que define tu script de Python
#include "known_locations_interfaces/srv/save_pose_location.hpp"

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Approach::Approach(const rclcpp::Node::SharedPtr &node,
                   const std::shared_ptr<Hardware> &hardware,
                   const std::shared_ptr<Vision> &vision,
                   const std::shared_ptr<Navigation> &navigation,
                   const std::shared_ptr<Knowledge> &knowledge)
    : node_(node), hardware_(hardware), vision_(vision), 
      navigation_(navigation), knowledge_(knowledge) 
{
    // Creamos el cliente de servicio dinámico para interactuar con tu script de Python
    save_loc_client_ = node_->create_client<known_locations_interfaces::srv::SavePoseLocation>(
        "locations_manager/save");
}

// =========================================================
// run
// =========================================================
Result Approach::run(const std::string &argument,
                     const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active())
    return {false, "FAILED", "goal_handle_inactive"};

  // ── READY ─────────────────────────────────────────────
  publishFeedback(gh, ApproachState::READY, argument);

  // ── LOADING ───────────────────────────────────────────
  publishFeedback(gh, ApproachState::LOADING, argument);

  std::string category = "guest";
  std::string index = "1";
  
  if (argument.find('_') != std::string::npos) {
      category = argument.substr(0, argument.find('_'));
      index = argument.substr(argument.find('_') + 1);
  } else {
      category = argument;
  }

  // Jalar la pose que guardó el FindPerson
  nlohmann::json raw_knowledge_response = knowledge_->get_world_state(category, index, "navigation");
  
  if (raw_knowledge_response.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[Approach] Target %s_%s not found, using last_person fallback...", category.c_str(), index.c_str());
    raw_knowledge_response = knowledge_->get_world_state("world", "last_person", "navigation");
    
    if (raw_knowledge_response.empty()) {
        publishFeedback(gh, ApproachState::FAILED, "target_not_in_knowledge: " + argument);
        return {false, "FAILED", "target_not_in_knowledge"};
    }
  }

  std::string serialized_nav_string = raw_knowledge_response.get<std::string>();
  nlohmann::json nav_array = nlohmann::json::parse(serialized_nav_string);

  double target_x = nav_array[0].get<double>();
  double target_y = nav_array[1].get<double>();

  // ── APPROACHING ───────────────────────────────────────
  publishFeedback(gh, ApproachState::APPROACHING, argument);

  const double R = 0.30; // 30 cm de distancia
  const std::string temp_location_name = "temp_approach_target";
  
  std::vector<double> angles_to_try = {0.0, 0.785, -0.785, 1.57, -1.57, 2.356, -2.356, 3.141};
  bool any_goal_reached = false;

  for (size_t i = 0; i < angles_to_try.size(); ++i) {
    if (!gh->is_active()) {
      publishFeedback(gh, ApproachState::FAILED, "goal_cancelled");
      return {false, "FAILED", "goal_cancelled"};
    }

    double alpha = angles_to_try[i];
    double goal_x = target_x + R * std::cos(alpha);
    double goal_y = target_y + R * std::sin(alpha);
    double goal_theta = std::atan2(target_y - goal_y, target_x - goal_x);

    std::string retry_detail = "retry_" + std::to_string(i + 1) + "_angle_" + 
                               std::to_string(static_cast<int>(alpha * 180.0 / M_PI)) + "_deg";
    publishFeedback(gh, ApproachState::APPROACHING, retry_detail);

    // ===================================================
    // ESCRITURA DIRECTA EN EL LOCATIONS MANAGER SERVER
    // ===================================================
    if (!save_loc_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(node_->get_logger(), "Service locations_manager/save not available!");
        continue;
    }

    auto save_req = std::make_shared<known_locations_interfaces::srv::SavePoseLocation::Request>();
    save_req->location_name = temp_location_name;
    save_req->pose.position.x = goal_x;
    save_req->pose.position.y = goal_y;
    save_req->pose.position.z = 0.0;
    save_req->pose.orientation.x = 0.0;
    save_req->pose.orientation.y = 0.0;
    save_req->pose.orientation.z = std::sin(goal_theta / 2.0);
    save_req->pose.orientation.w = std::cos(goal_theta / 2.0);

    auto save_future = save_loc_client_->async_send_request(save_req);
    if (save_future.wait_for(2s) != std::future_status::ready || !save_future.get()->success) {
        RCLCPP_WARN(node_->get_logger(), "[Approach] Failed to inject pose into Python server.");
        continue;
    }

    // Le damos un instante al Broadcaster de TFs del server de Python para asentar el frame
    std::this_thread::sleep_for(150ms);

    // 4. Mandamos al GoTo Detailed nativo a ejecutar la navegación global
    auto nav_result = navigation_->GoToDetailed(temp_location_name, 45s);

    if (nav_result.success) {
        any_goal_reached = true;
        break;
    }

    RCLCPP_WARN(node_->get_logger(), "[Approach] Angle %.1f deg blocked by Nav2. Trying next option...", alpha * 180.0 / M_PI);
    std::this_thread::sleep_for(100ms);
  }

  // ── SUCCESS / FAILED ──────────────────────────────────
  if (any_goal_reached) {
    publishFeedback(gh, ApproachState::SUCCESS, argument);
    return {true, "SUCCESS", "approached_safely: " + argument};
  } else {
    publishFeedback(gh, ApproachState::FAILED, "all_angles_blocked: " + argument);
    return {false, "FAILED", "all_approach_angles_blocked: " + argument};
  }
}

// =========================================================
// Métodos Auxiliares
// =========================================================
void Approach::publishFeedback(const std::shared_ptr<GoalHandle>& gh, ApproachState state, const std::string& detail) {
    if (!gh->is_active()) return;
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