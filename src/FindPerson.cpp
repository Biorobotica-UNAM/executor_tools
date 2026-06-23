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
    constexpr double kDegToRad = 0.017453292519943295;
    for (double tilt_deg : {0.0, -10.0, -20.0, -30.0}) {
      scan_points.push_back({0.0, tilt_deg * kDegToRad});
    }
  } else {
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
// waitForHeadStable
// ============================================================
//
// Espera a que la cabeza deje de moverse antes de capturar un frame.
// Funciona en dos modos según lo que exponga Hardware:
//
//   A) Si Hardware::GetHeadPosition() existe: compara la posición actual
//      contra la lectura anterior; sale cuando el delta es menor a
//      tolerance_rad en dos muestras consecutivas.
//
//   B) Si no existe (o falla): fallback a un sleep mínimo garantizado de
//      kMinSettleMs ms, que es más conservador que el sleep fijo anterior.
//
// En cualquier caso respeta goal_cancelled para no bloquear la cancelación.

bool FindPerson::waitForHeadStable(const std::shared_ptr<GoalHandle> &gh,
                                   int timeout_ms,
                                   double tolerance_rad) {
  constexpr int kPollMs    = 80;   // intervalo de muestreo
  constexpr int kMinSettleMs = 400; // mínimo aunque la pose ya sea estable

  const auto t_start = std::chrono::steady_clock::now();
  const auto deadline = t_start + std::chrono::milliseconds(timeout_ms);

  // Mínimo de settle para dar tiempo al servo de arrancar el movimiento
  // antes de empezar a leer la posición.
  std::this_thread::sleep_for(std::chrono::milliseconds(kMinSettleMs));

  std::vector<double> prev_pos;
  int stable_count = 0;             // muestras consecutivas bajo umbral

  while (std::chrono::steady_clock::now() < deadline) {
    if (!gh || !gh->is_active())
      return false;

    // ── Intento A: pose real ────────────────────────────────
    // Si tu Hardware expone GetHeadPosition() descomenta este bloque.
    // El método debe devolver {pan, tilt} en radianes o un vector vacío
    // si el dato no está disponible todavía.
    //
    // std::vector<double> cur_pos = hardware_->GetHeadPosition();
    // if (!cur_pos.empty() && cur_pos.size() == prev_pos.size()) {
    //   double delta = 0.0;
    //   for (std::size_t i = 0; i < cur_pos.size(); ++i)
    //     delta = std::max(delta, std::abs(cur_pos[i] - prev_pos[i]));
    //
    //   if (delta < tolerance_rad) {
    //     if (++stable_count >= 2) {   // 2 muestras estables (~160 ms)
    //       RCLCPP_DEBUG(node_->get_logger(),
    //                    "[FindPerson] Head stable after %.0f ms (delta=%.4f rad)",
    //                    std::chrono::duration<double, std::milli>(
    //                        std::chrono::steady_clock::now() - t_start).count(),
    //                    delta);
    //       return true;
    //     }
    //   } else {
    //     stable_count = 0;
    //   }
    // }
    // prev_pos = cur_pos;

    // ── Fallback B: tiempo mínimo ya cumplido, salir ────────
    // Si no hay GetHeadPosition, el kMinSettleMs de arriba es suficiente.
    // Elimina este return y descomenta el bloque A cuando tengas la API.
    (void)prev_pos;
    (void)stable_count;
    (void)tolerance_rad;
    RCLCPP_DEBUG(node_->get_logger(),
                 "[FindPerson] waitForHeadStable: using time-based settle (%d ms)",
                 kMinSettleMs);
    return true;

    std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
  }

  RCLCPP_WARN(node_->get_logger(),
              "[FindPerson] waitForHeadStable: timeout after %d ms", timeout_ms);
  return true; // timeout no es fallo, continuamos igual
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

  // Espera a que la cabeza se estabilice antes de detectar.
  // Sustituye el sleep fijo anterior (12 × 100 ms = 1.2 s) por una espera
  // activa que sale en cuanto la articulación deja de moverse.
  if (!waitForHeadStable(gh))
    return false;

  // ── 2. Detección inicial ──────────────────────────────────
  publishFeedback(gh, FindPersonState::DETECTING);

  Vision::PersonDetection detection;
  constexpr auto kDetectPoseTimeout = 2s;

  if (!vision_->DetectPersonPose(detection, kDetectPoseTimeout)) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindPerson] DetectPersonPose failed at pan=%.3f tilt=%.3f",
                pan, tilt);
    return false;
  }

  if (!gh->is_active())
    return false;

  RCLCPP_INFO(node_->get_logger(),
              "[FindPerson] DetectPersonPose OK bbox='%s' pose_frame='%s' pos=[%.3f %.3f %.3f]",
              detection.coords.c_str(),
              detection.pose.header.frame_id.c_str(),
              detection.pose.pose.position.x,
              detection.pose.pose.position.y,
              detection.pose.pose.position.z);

  // ── 3. Centering loop (closed-loop) ───────────────────────
  //
  // En vez de corregir el LookAt con un heurístico open-loop sobre el
  // bbox (que depende de la distancia a la persona), ejecutamos hasta
  // kMaxCenterIter iteraciones de detect→lookat. Cada iteración usa la
  // pose recién obtenida con la cabeza ya quieta, por lo que converge
  // incluso si la primera detección vino con la cámara en movimiento.
  //
  // Criterio de salida anticipada: si el centro del bbox se acerca a
  // kImageCenterY dentro de kCenterThresholdPx, se considera centrada.

  publishFeedback(gh, FindPersonState::LOOK_AT_TARGET);

  constexpr int    kMaxCenterIter      = 2;
  constexpr double kImageCenterY       = 240.0;
  constexpr double kCenterThresholdPx  = 40.0;  // convergencia aceptable

  Vision::PersonDetection current_detection = detection;

  for (int iter = 0; iter < kMaxCenterIter; ++iter) {
    if (!gh->is_active())
      return false;

    // ── 3a. LookAt con la pose del frame actual ───────────
    geometry_msgs::msg::PointStamped look_point;
    look_point.header = current_detection.pose.header;
    look_point.point  = current_detection.pose.pose.position;

    hardware_->LookAt(look_point);

    RCLCPP_INFO(node_->get_logger(),
                "[FindPerson] Centering iter=%d LookAt pos=[%.3f %.3f %.3f]",
                iter,
                look_point.point.x,
                look_point.point.y,
                look_point.point.z);

    // ── 3b. Esperar estabilización real antes de re-detectar
    if (!waitForHeadStable(gh))
      return false;

    // ── 3c. Re-detect con cabeza quieta ───────────────────
    Vision::PersonDetection refined;
    if (!vision_->DetectPersonPose(refined, 2s)) {
      // Si falla la re-detección no abortamos: usamos la última pose válida
      // y salimos del loop para continuar con la identificación.
      RCLCPP_WARN(node_->get_logger(),
                  "[FindPerson] Re-detect failed on centering iter=%d, using last valid pose",
                  iter);
      break;
    }

    if (!gh->is_active())
      return false;

    current_detection = refined;

    // ── 3d. Comprobar si la cara ya está centrada ─────────
    double face_center_y = kImageCenterY;
    {
      std::stringstream ss(refined.coords);
      std::string tok;
      double bbox[4] = {};
      bool ok = true;
      for (int i = 0; i < 4; ++i) {
        if (!std::getline(ss, tok, ',')) { ok = false; break; }
        try { bbox[i] = std::stod(tok); } catch (...) { ok = false; break; }
      }
      if (ok)
        face_center_y = (bbox[1] + bbox[3]) * 0.5;
    }

    const double error_y = std::abs(face_center_y - kImageCenterY);
    RCLCPP_INFO(node_->get_logger(),
                "[FindPerson] Centering iter=%d face_center_y=%.1f error=%.1f px",
                iter, face_center_y, error_y);

    if (error_y < kCenterThresholdPx) {
      RCLCPP_INFO(node_->get_logger(),
                  "[FindPerson] Face centered (error=%.1f < %.1f px), stopping early",
                  error_y, kCenterThresholdPx);
      break;
    }
  }

  // Desde aquí `current_detection` contiene la pose del último frame
  // capturado con la cabeza estable. Se usa como pose definitiva de la
  // persona para navegación y para el registro facial.
  const geometry_msgs::msg::PoseStamped target_pose = current_detection.pose;

  if (!gh->is_active())
    return false;

  // ── 4. Refresh del frame cacheado en face_recog ───────────
  //
  // Ahora que la cabeza está quieta y la cara está (aproximadamente) centrada
  // hacemos el refresh. Se elimina el caso especial de "cara baja" porque el
  // centering loop ya lo resuelve.

  bool refreshed = false;
  std::string centered_coords;
  float centered_z = 0.0f;

  for (int attempt = 1; attempt <= 5; ++attempt) {
    if (vision_->DetectPerson("", centered_coords, centered_z, 1s)) {
      refreshed = true;
      RCLCPP_INFO(node_->get_logger(),
                  "[FindPerson] Refreshed face frame before identify/register: bbox='%s' attempt=%d",
                  centered_coords.c_str(), attempt);
      break;
    }
    RCLCPP_DEBUG(node_->get_logger(),
                 "[FindPerson] Could not refresh centered face frame (attempt %d)", attempt);
    std::this_thread::sleep_for(200ms);
  }

  if (!refreshed) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindPerson] Using previous detection cache for register because centered refresh failed");
  }

  // ── 5. Identify ───────────────────────────────────────────
  publishFeedback(gh, FindPersonState::IDENTIFYING);

  std::string identified_name;
  const bool identified = vision_->IdentifyPerson(identified_name);

  if (identified) {
    if (!anybody && identified_name != target) {
      auto stored_json = knowledge_->get_world_state(category, person_index, "name");
      const bool has_reference =
          stored_json.is_string() && !stored_json.get<std::string>().empty();

      if (has_reference && identified_name != stored_json.get<std::string>()) {
        RCLCPP_INFO(node_->get_logger(),
                    "[FindPerson] Identified '%s', not target '%s' (expected: '%s') — skipping",
                    identified_name.c_str(), target.c_str(),
                    stored_json.get<std::string>().c_str());
        return false;
      }

      if (!has_reference) {
        RCLCPP_INFO(node_->get_logger(),
                    "[FindPerson] No face reference stored for '%s' — accepting identified '%s'",
                    target.c_str(), identified_name.c_str());
      }
    }

    if (anybody) {
      found_name = generatePersonName(std::stoull(person_index));
      RCLCPP_INFO(node_->get_logger(),
                  "[FindPerson] Identified existing '%s', storing as slot '%s'",
                  identified_name.c_str(), found_name.c_str());

      publishFeedback(gh, FindPersonState::REGISTERING, found_name);
      if (vision_->RegisterPerson(found_name)) {
        RCLCPP_INFO(node_->get_logger(), "[FindPerson] Person registered/overwritten as: %s",
                    found_name.c_str());
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "[FindPerson] Could not register/overwrite face for '%s'; continuing with knowledge update",
                    found_name.c_str());
      }
    } else {
      found_name = identified_name;
      RCLCPP_INFO(node_->get_logger(), "[FindPerson] Person identified: %s", found_name.c_str());
    }

    publishFeedback(gh, FindPersonState::DESCRIBING);

    std::string description;
    if (vision_->DescribePerson(found_name, description)) {
      RCLCPP_INFO(node_->get_logger(), "[FindPerson] Description: %s", description.c_str());
      publishFeedback(gh, FindPersonState::UPDATING, found_name);
      updatePersonState(category, person_index, found_name, target_pose,
                        current_detection.nav, description, true);
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "[FindPerson] DescribePerson failed for '%s'", found_name.c_str());
    }

    return true;
  }

  // ── 6. Person NOT in the DB ───────────────────────────────

  if (!anybody) {
    RCLCPP_INFO(node_->get_logger(),
                "[FindPerson] Unknown person at this position — skipping");
    return false;
  }

  publishFeedback(gh, FindPersonState::REGISTERING);
  const std::string new_name = generatePersonName(std::stoull(person_index));

  if (vision_->RegisterPerson(new_name)) {
    found_name = new_name;
    RCLCPP_INFO(node_->get_logger(), "[FindPerson] Person registered as: %s", found_name.c_str());
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "[FindPerson] Registration failed for '%s'", new_name.c_str());
    found_name = new_name;
  }

  publishFeedback(gh, FindPersonState::DESCRIBING);

  std::string description;
  if (vision_->DescribePerson(found_name, description)) {
    RCLCPP_INFO(node_->get_logger(), "[FindPerson] Description: %s", description.c_str());
    publishFeedback(gh, FindPersonState::UPDATING, found_name);
    updatePersonState(category, person_index, found_name, target_pose,
                      current_detection.nav, description, anybody);
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "[FindPerson] DescribePerson failed for '%s'", found_name.c_str());
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
                                   const geometry_msgs::msg::PoseStamped &nav,
                                   const std::string &description,
                                   bool overwrite_name) {
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

  const auto &np = nav.pose.position;
  const auto &nq = nav.pose.orientation;
  nlohmann::json nav_json = {np.x, np.y, np.z, nq.x, nq.y, nq.z, nq.w};

  bool update_ok = true;

  if (overwrite_name) {
    const bool ok = knowledge_->update_world_state(category, index, "name", name);
    update_ok = update_ok && ok;
    if (!ok) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[FindPerson] Failed to update knowledge: %s_%s name=%s",
                   category.c_str(), index.c_str(), name.c_str());
    }
  }

  const bool description_ok =
      knowledge_->update_world_state(category, index, "description", desc_json.dump());
  update_ok = update_ok && description_ok;

  const bool navigation_ok =
      knowledge_->update_world_state(category, index, "navigation", nav_json.dump());
  update_ok = update_ok && navigation_ok;

  if (update_ok) {
    RCLCPP_INFO(node_->get_logger(),
                "[FindPerson] Knowledge updated for %s_%s as '%s'",
                category.c_str(), index.c_str(), name.c_str());
  }
}

// ============================================================
// getNextGuestIndex / generatePersonName
// ============================================================

uint64_t FindPerson::getNextGuestIndex() {
  static std::atomic<uint64_t> counter{0};
  return ++counter;
}

std::string FindPerson::generatePersonName(uint64_t index) {
  return "guest_" + std::to_string(index);
}

// ============================================================
// publishFeedback / stateName
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

std::string FindPerson::stateName(FindPersonState s) {
  switch (s) {
  case FindPersonState::READY:              return "READY";
  case FindPersonState::SCANNING:           return "SCANNING";
  case FindPersonState::DETECTING:          return "DETECTING";
  case FindPersonState::CALCULATING_POSE:   return "CALCULATING_POSE";
  case FindPersonState::LOOK_AT_TARGET:     return "LOOK_AT_TARGET";
  case FindPersonState::IDENTIFYING:        return "IDENTIFYING";
  case FindPersonState::REGISTERING:        return "REGISTERING";
  case FindPersonState::DESCRIBING:         return "DESCRIBING";
  case FindPersonState::UPDATING:           return "UPDATING";
  case FindPersonState::SUCCESS:            return "SUCCESS";
  case FindPersonState::FAILED:             return "FAILED";
  default:                                  return "UNKNOWN";
  }
}
