#include "executor_tools/FindPerson.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

// ============================================================
// Constructor
// ============================================================

FindPerson::FindPerson(const rclcpp::Node::SharedPtr &node,
                       const std::shared_ptr<Hardware> &hardware,
                       const std::shared_ptr<Vision> &vision,
                       const std::shared_ptr<Knowledge> &knowledge)
    : node_(node), hardware_(hardware), vision_(vision), knowledge_(knowledge) {
}

// ============================================================
// run
// ============================================================

Result FindPerson::run(const std::string &argument,
                       const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active())
    return {false, "FAILED", "goal_handle_inactive"};

  publishFeedback(gh, FindPersonState::READY, argument);

  const bool anybody = argument.rfind("anybody", 0) == 0;

  std::string category;
  std::string person_index;

  if (anybody) {
    category = "guest";
    const std::string prefix = "anybody_";
    if (argument.rfind(prefix, 0) == 0 && argument.size() > prefix.size()) {
      person_index = argument.substr(prefix.size());
    } else {
      person_index = std::to_string(getNextGuestIndex());
    }
  } else {
    const std::size_t sep = argument.rfind('_');

    if (sep == std::string::npos) {
      category = argument;
      person_index = "1";
    } else {
      category = argument.substr(0, sep);
      person_index = argument.substr(sep + 1);
    }
  }

  RCLCPP_INFO(node_->get_logger(),
              "[FindPerson] argument='%s' anybody=%s category='%s' index='%s'",
              argument.c_str(), anybody ? "true" : "false",
              category.c_str(), person_index.c_str());

  // ── Scan positions ───────────────────────────────────────

  struct ScanPoint {
    double pan;
    double tilt;
  };
  std::vector<ScanPoint> scan_points;

  if (anybody) {
    // Para invitados nuevos no se barre lateralmente: se busca al frente
    // bajando la cabeza por escalones para encontrar caras bajas en la imagen.
    constexpr double kDegToRad = 0.017453292519943295;
    for (double tilt_deg : {0.0, -10.0, -20.0, -30.0}) {
      scan_points.push_back({0.0, tilt_deg * kDegToRad});
    }
  } else {
    // Para una persona conocida se hacen dos barridos laterales: primero con
    // la cabeza recta y luego con la cabeza baja. No se repite el centro al
    // final; si no se encuentra a nadie, run() regresa la cabeza a reposo.
    constexpr double kDegToRad = 0.017453292519943295;
    for (double tilt_deg : {0.0, -25.0}) {
      for (double pan : {0.0, -0.6, 0.6}) {
        scan_points.push_back({pan, tilt_deg * kDegToRad});
      }
    }
  }

  // ── Scanning loop ────────────────────────────────────────

  for (const auto &sp : scan_points) {
    if (!gh->is_active())
      return {false, "FAILED", "goal_cancelled"};

    std::string found_name;

    const bool found = scanPosition(sp.pan, sp.tilt, anybody, argument,
                                    category, person_index, found_name, gh);

    if (!gh->is_active())
      return {false, "FAILED", "goal_cancelled"};

    if (found) {
      publishFeedback(gh, FindPersonState::SUCCESS, found_name);
      return {true, "SUCCESS", "person_found: " + found_name};
    }
  }

  // ── Nobody found after full sweep ────────────────────────

  hardware_->MoveHead({0.0, 0.0});
  publishFeedback(gh, FindPersonState::FAILED, argument);
  return {false, "FAILED", "person_not_found: " + argument};
}

// ============================================================
// scanPosition
// ============================================================

bool FindPerson::scanPosition(double pan, double tilt, bool anybody,
                              const std::string &target,
                              const std::string &category,
                              const std::string &person_index,
                              std::string &found_name,
                              const std::shared_ptr<GoalHandle> &gh) {
  // ── 1. Move head ──────────────────────────────────────────
  publishFeedback(gh, FindPersonState::SCANNING);
  RCLCPP_INFO(node_->get_logger(),
              "[FindPerson] scanPosition pan=%.3f tilt=%.3f (%.1f deg) target='%s'",
              pan, tilt, tilt * 57.29577951308232, target.c_str());
  hardware_->MoveHead({pan, tilt});

  for (int i = 0; i < 12; ++i) {
    std::this_thread::sleep_for(100ms);
    if (!gh->is_active())
      return false;
  }

  // ── 2. Detect + pose ──────────────────────────────────────
  publishFeedback(gh, FindPersonState::DETECTING);

  Vision::PersonDetection detection;
  // Timeout intencionalmente explicito: evita depender del valor por defecto
  // del wrapper Vision y hace visible cuanto dura cada posicion de escaneo.
  constexpr auto kDetectPoseTimeout = 2s;

  RCLCPP_INFO(node_->get_logger(),
              "[FindPerson] DetectPersonPose search timeout=%ld s",
              kDetectPoseTimeout.count());

  if (!vision_->DetectPersonPose(detection, kDetectPoseTimeout)) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindPerson] DetectPersonPose failed at pan=%.3f tilt=%.3f",
                pan, tilt);
    return false;
  }

  if (!gh->is_active())
    return false;

  geometry_msgs::msg::PoseStamped target_pose = detection.pose;
  RCLCPP_INFO(node_->get_logger(),
              "[FindPerson] DetectPersonPose OK bbox='%s' pose_frame='%s' pos=[%.3f %.3f %.3f]",
              detection.coords.c_str(), target_pose.header.frame_id.c_str(),
              target_pose.pose.position.x, target_pose.pose.position.y,
              target_pose.pose.position.z);

  if (!gh->is_active())
    return false;

  // ── 3. Look at the person ─────────────────────────────────
  publishFeedback(gh, FindPersonState::LOOK_AT_TARGET);

  geometry_msgs::msg::PointStamped look_point;
  look_point.header = target_pose.header;
  look_point.point = target_pose.pose.position;

  double face_center_y = 0.0;
  bool face_center_parsed = false;
  {
    std::stringstream bbox_stream(detection.coords);
    std::string token;
    double bbox[4] = {0.0, 0.0, 0.0, 0.0};
    bool parsed = true;

    for (int i = 0; i < 4; ++i) {
      if (!std::getline(bbox_stream, token, ',')) {
        parsed = false;
        break;
      }

      try {
        bbox[i] = std::stod(token);
      } catch (const std::exception &) {
        parsed = false;
        break;
      }
    }

    if (parsed) {
      face_center_y = (bbox[1] + bbox[3]) * 0.5;
      face_center_parsed = true;
      constexpr double kImageCenterY = 240.0;
      constexpr double kMaxLookZAdjust = 0.15;
      const double normalized_error =
          (face_center_y - kImageCenterY) / kImageCenterY;
      const double look_z_adjust =
          std::clamp(normalized_error * 0.18, -kMaxLookZAdjust,
                     kMaxLookZAdjust);

      // Si la cara aparece baja en la imagen, apuntamos un poco mas abajo para
      // que suba hacia el centro antes de guardar el frame de registro.
      look_point.point.z -= look_z_adjust;
      RCLCPP_INFO(node_->get_logger(),
                  "[FindPerson] Centering face before capture: bbox_center_y=%.1f look_z_adjust=%.3f adjusted_z=%.3f",
                  face_center_y, look_z_adjust, look_point.point.z);
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "[FindPerson] Could not parse bbox '%s' for centering",
                  detection.coords.c_str());
    }
  }

  hardware_->LookAt(look_point);

  for (int i = 0; i < 15; ++i) {
    std::this_thread::sleep_for(100ms);
    if (!gh->is_active())
      return false;
  }

  if (face_center_parsed && face_center_y > 300.0) {
    // La cara suele quedar pegada al borde inferior. Antes de registrar el
    // rostro bajamos un poco mas la cabeza y luego refrescamos el cache de
    // deteccion para guardar una captura mas centrada.
    constexpr double kCaptureTiltDown = 0.25;
    const double capture_tilt = tilt - kCaptureTiltDown;
    RCLCPP_INFO(
        node_->get_logger(),
        "[FindPerson] Face is low in image (center_y=%.1f); lowering head for capture: pan=%.3f tilt=%.3f",
        face_center_y, pan, capture_tilt);
    hardware_->MoveHead({pan, capture_tilt});

    for (int i = 0; i < 10; ++i) {
      std::this_thread::sleep_for(100ms);
      if (!gh->is_active())
        return false;
    }
  }

  std::string centered_coords;
  float centered_z = 0.0f;
  bool refreshed = false;
  // Este refresh actualiza el frame cacheado en face_recog. Si falla, se usa el
  // ultimo frame valido para no perder el registro de la persona detectada.
  for (int attempt = 1; attempt <= 5; ++attempt) {
    if (vision_->DetectPerson("", centered_coords, centered_z, 1s)) {
      refreshed = true;
      RCLCPP_INFO(
          node_->get_logger(),
          "[FindPerson] Refreshed face frame before identify/register: bbox='%s' attempt=%d",
          centered_coords.c_str(), attempt);
      break;
    }

    RCLCPP_DEBUG(
        node_->get_logger(),
        "[FindPerson] Could not refresh centered face frame before register (attempt %d)",
        attempt);
    std::this_thread::sleep_for(200ms);
  }

  if (!refreshed) {
    RCLCPP_WARN(
        node_->get_logger(),
        "[FindPerson] Using previous detection cache for register because centered refresh failed");
  }

  // ── 4. Identify ───────────────────────────────────────────
  publishFeedback(gh, FindPersonState::IDENTIFYING);

  std::string identified_name;
  const bool identified = vision_->IdentifyPerson(identified_name);

  if (identified) {
    if (!anybody && identified_name != target) {
      RCLCPP_INFO(node_->get_logger(),
                  "[FindPerson] Identified '%s', not target '%s' — skipping",
                  identified_name.c_str(), target.c_str());
      return false;
    }

    if (anybody) {
      found_name = generatePersonName(std::stoull(person_index));
      RCLCPP_INFO(
          node_->get_logger(),
          "[FindPerson] Identified existing '%s', but storing current scan as slot '%s'",
          identified_name.c_str(), found_name.c_str());

      publishFeedback(gh, FindPersonState::REGISTERING, found_name);
      if (vision_->RegisterPerson(found_name)) {
        RCLCPP_INFO(node_->get_logger(),
                    "[FindPerson] Person registered/overwritten as: %s",
                    found_name.c_str());
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "[FindPerson] Could not register/overwrite face for '%s'; continuing with knowledge update",
                    found_name.c_str());
      }
    } else {
      found_name = identified_name;
      RCLCPP_INFO(node_->get_logger(), "[FindPerson] Person identified: %s",
                  found_name.c_str());
    }

    publishFeedback(gh, FindPersonState::DESCRIBING);

    std::string description;

    if (vision_->DescribePerson(found_name, description)) {
      RCLCPP_INFO(node_->get_logger(), "[FindPerson] Description: %s",
                  description.c_str());
      publishFeedback(gh, FindPersonState::UPDATING, found_name);
      updatePersonState(category, person_index, found_name, target_pose,
                        description, anybody);
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "[FindPerson] DescribePerson failed for '%s'",
                  found_name.c_str());
    }

    return true;
  }

  // ── 5. Person NOT in the DB ───────────────────────────────

  if (!anybody) {
    RCLCPP_INFO(node_->get_logger(),
                "[FindPerson] Unknown person at this position — skipping");
    return false;
  }

  publishFeedback(gh, FindPersonState::REGISTERING);
  const std::string new_name = generatePersonName(std::stoull(person_index));

  if (vision_->RegisterPerson(new_name)) {
    found_name = new_name;
    RCLCPP_INFO(node_->get_logger(), "[FindPerson] Person registered as: %s",
                found_name.c_str());
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "[FindPerson] Registration failed for '%s'", new_name.c_str());
    found_name = new_name;
  }

  publishFeedback(gh, FindPersonState::DESCRIBING);

  std::string description;

  if (vision_->DescribePerson(found_name, description)) {
    RCLCPP_INFO(node_->get_logger(), "[FindPerson] Description: %s",
                description.c_str());
    publishFeedback(gh, FindPersonState::UPDATING, found_name);
    updatePersonState(category, person_index, found_name, target_pose,
                      description, anybody);
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "[FindPerson] DescribePerson failed for '%s'",
                found_name.c_str());
  }

  return true;
}

// ============================================================
// updatePersonState
// ============================================================

void FindPerson::updatePersonState(const std::string &category,
                                   const std::string &index,
                                   const std::string &name,
                                   const geometry_msgs::msg::PoseStamped &pose,
                                   const std::string &description,
                                   bool overwrite_name) {
  // description llega como CSV ("male, black shirt, young, ...")
  // se convierte a JSON array para el servicio
  std::vector<std::string> tokens;
  std::stringstream ss(description);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token.erase(0, token.find_first_not_of(" "));
    token.erase(token.find_last_not_of(" ") + 1);
    if (!token.empty())
      tokens.push_back(token);
  }

  nlohmann::json desc_json = tokens;

  const auto &p = pose.pose.position;
  const auto &q = pose.pose.orientation;
  nlohmann::json nav = {p.x, p.y, p.z, q.x, q.y, q.z, q.w};

  bool update_ok = true;

  // Este guardado es el estado semantico de la persona. Es independiente del
  // archivo facial que register_person guarda en ~/vision/faces_detected.
  if (overwrite_name) {
    const bool ok =
        knowledge_->update_world_state(category, index, "name", name);
    update_ok = update_ok && ok;
    if (!ok) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[FindPerson] Failed to update knowledge: %s_%s name=%s",
                   category.c_str(), index.c_str(), name.c_str());
    }
  }

  const bool description_ok =
      knowledge_->update_world_state(category, index, "description",
                                     desc_json.dump());
  update_ok = update_ok && description_ok;
  if (!description_ok) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[FindPerson] Failed to update knowledge: %s_%s description",
                 category.c_str(), index.c_str());
  }

  const bool navigation_ok =
      knowledge_->update_world_state(category, index, "navigation", nav.dump());
  update_ok = update_ok && navigation_ok;
  if (!navigation_ok) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[FindPerson] Failed to update knowledge: %s_%s navigation",
                 category.c_str(), index.c_str());
  }

  if (update_ok) {
    RCLCPP_INFO(node_->get_logger(),
                "[FindPerson] Knowledge updated for %s_%s as '%s'",
                category.c_str(), index.c_str(), name.c_str());
  }
}

// ============================================================
// getNextGuestIndex
// ============================================================

uint64_t FindPerson::getNextGuestIndex() {
  static std::atomic<uint64_t> counter{0};
  return ++counter;
}

// ============================================================
// generatePersonName
// ============================================================

std::string FindPerson::generatePersonName(uint64_t index) {
  return "guest_" + std::to_string(index);
}

// ============================================================
// publishFeedback
// ============================================================

void FindPerson::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                                 FindPersonState state,
                                 const std::string &detail) {
  if (!gh || !gh->is_active())
    return;

  std::string msg = "[find_person] " + stateName(state);
  if (!detail.empty())
    msg += ": " + detail;

  auto fb = std::make_shared<ExecuteTask::Feedback>();
  fb->feedback.message = msg;
  gh->publish_feedback(fb);

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// ============================================================
// stateName
// ============================================================

std::string FindPerson::stateName(FindPersonState s) {
  switch (s) {
  case FindPersonState::READY:
    return "READY";
  case FindPersonState::SCANNING:
    return "SCANNING";
  case FindPersonState::DETECTING:
    return "DETECTING";
  case FindPersonState::CALCULATING_POSE:
    return "CALCULATING_POSE";
  case FindPersonState::LOOK_AT_TARGET:
    return "LOOK_AT_TARGET";
  case FindPersonState::IDENTIFYING:
    return "IDENTIFYING";
  case FindPersonState::REGISTERING:
    return "REGISTERING";
  case FindPersonState::DESCRIBING:
    return "DESCRIBING";
  case FindPersonState::UPDATING:
    return "UPDATING";
  case FindPersonState::SUCCESS:
    return "SUCCESS";
  case FindPersonState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}
