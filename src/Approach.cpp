#include "executor_tools/Approach.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

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
{}

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

  // Resolver Categoría e Índice desde el argumento (ej. "guest_1")
  std::string category = "guest";
  std::string index = "1";
  
  if (argument.find('_') != std::string::npos) {
      category = argument.substr(0, argument.find('_'));
      index = argument.substr(argument.find('_') + 1);
  } else {
      category = argument;
  }

  // Interrogar al Knowledge Bank por la pose serializada en nav.dump()
  nlohmann::json raw_knowledge_response = knowledge_->get_world_state(category, index, "navigation");
  
  if (raw_knowledge_response.empty()) {
    publishFeedback(gh, ApproachState::FAILED, "target_not_in_knowledge: " + argument);
    return {false, "FAILED", "target_not_in_knowledge"};
  }

  // Desempaquetar el string JSON interno hacia el array numérico real
  std::string serialized_nav_string = raw_knowledge_response.get<std::string>();
  nlohmann::json nav_array = nlohmann::json::parse(serialized_nav_string);

  if (!nav_array.is_array() || nav_array.size() != 7) {
    publishFeedback(gh, ApproachState::FAILED, "invalid_serialized_pose_format: " + argument);
    return {false, "FAILED", "invalid_pose_format"};
  }

  double target_x = nav_array[0].get<double>();
  double target_y = nav_array[1].get<double>();

  // ── APPROACHING (Muestreo Circular a 45° con Nav2) ────
  publishFeedback(gh, ApproachState::APPROACHING, argument);

  const double R = 0.30; // Radio de detención seguro (30 cm)
  
  // Vector de ángulos en abanico (0°, +45°, -45°, +90°, -90°, +135°, -135°, 180°)
  std::vector<double> angles_to_try = {0.0, -0.785, 0.785, -1.57, 1.57, -2.356, 2.356, 3.141};
  bool any_goal_reached = false;

  for (size_t i = 0; i < angles_to_try.size(); ++i) {
    if (!gh->is_active()) {
      publishFeedback(gh, ApproachState::FAILED, "goal_cancelled");
      return {false, "FAILED", "goal_cancelled"};
    }

    double alpha = angles_to_try[i];
    
    // Ecuaciones de la circunferencia para desplazar la meta de Nav2
    double goal_x = target_x + R * std::cos(alpha);
    double goal_y = target_y + R * std::sin(alpha);
    
    // Calcular orientación cartesiana para que Justina mire al objetivo central
    double goal_theta = std::atan2(target_y - goal_y, target_x - goal_x);

    std::string retry_detail = "retry_" + std::to_string(i + 1) + "_angle_" + 
                               std::to_string(static_cast<int>(alpha * 180.0 / M_PI)) + 
                               "_deg";
    publishFeedback(gh, ApproachState::APPROACHING, retry_detail);

    // Invocar la navegación global síncrona
    bool nav_ok = navigation_->GoToCoordinates(goal_x, goal_y, goal_theta);

    if (nav_ok) {
        any_goal_reached = true;
        break; // Éxito: Rompemos el abanico cinemático
    }

    RCLCPP_WARN(node_->get_logger(), "[Approach] Angle %.1f deg blocked by obstacles. Trying next offset...", alpha * 180.0 / M_PI);
    std::this_thread::sleep_for(200ms);
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
// publishFeedback
// =========================================================
void Approach::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                               ApproachState                      state,
                               const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[approach] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string Approach::stateName(ApproachState s)
{
    switch (s) {
        case ApproachState::READY:       return "READY";
        case ApproachState::LOADING:     return "LOADING";
        case ApproachState::APPROACHING: return "APPROACHING";
        case ApproachState::SUCCESS:     return "SUCCESS";
        case ApproachState::FAILED:      return "FAILED";
        default:                         return "UNKNOWN";
    }
}