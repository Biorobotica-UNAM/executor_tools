#include "executor_tools/FindSpace.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <optional>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

namespace {
double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromTransform(const geometry_msgs::msg::TransformStamped &tf) {
  const auto &q = tf.transform.rotation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

void FindSpace::configureShelfPreference(const SpaceTarget &space_target) {
  const json left_object = knowledge_->get_world_state("robot", "", "left_hand");
  const json right_object =
      knowledge_->get_world_state("robot", "", "right_hand");

  double preferred_y = 0.0;
  bool has_hand_preference = false;
  bool has_semantic_preference = false;
  std::string semantic_detail;

  if (!left_object.is_string() || !right_object.is_string()) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] no se pudo leer ocupacion de manos; se conserva "
                "preferred_y_m de launch para shelf");
  } else {
    const bool left_occupied = left_object.get<std::string>() != "nothing";
    const bool right_occupied = right_object.get<std::string>() != "nothing";

    if (!left_occupied && !right_occupied) {
      RCLCPP_INFO(node_->get_logger(),
                  "[FindSpace] ninguna mano ocupada; se conserva preferred_y_m "
                  "de launch para shelf");
    } else {
      // En base_link, +Y es izquierda del robot y -Y es derecha.
      const bool prefer_right = right_occupied || !left_occupied;
      preferred_y = prefer_right ? -0.18 : 0.18;
      has_hand_preference = true;
    }
  }
  // Si hay un objeto en mano, intentamos usar su categoria para preferir un lado(category)
  const auto categories = loadObjectCategories();
  const auto held_object = heldObjectName();
  if (!held_object) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] no se encontro objeto en mano para preferencia "
                "semantica de shelf");
  } else if (categories.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] diccionario de objetos vacio/no disponible; "
                "se usara preferencia por mano si existe");
  } else {
    const auto held_category = categoryForObject(*held_object, categories);
    if (!held_category) {
      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] objeto en mano '%s' no tiene categoria en "
                  "diccionario; se usara preferencia por mano si existe",
                  held_object->c_str());
    } else {
      std::vector<std::string> category_objects;
      const std::string normalized_held = normalizeObjectToken(*held_object);
      for (const auto &object : categories.at(*held_category)) {
        if (object != normalized_held) {
          category_objects.push_back(object);
        }
      }
      const auto category_y =
          preferredShelfYFromCategory(*held_category, category_objects);
      if (category_y) {
        preferred_y = *category_y;
        has_semantic_preference = true;
        semantic_detail = "held='" + *held_object + "' category='" +
                          *held_category + "'";
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "[FindSpace] no se pudo detectar lado de shelf para "
                    "categoria '%s'; se usara preferencia por mano si existe",
                    held_category->c_str());
      }
    }
  }

  rclcpp::AsyncParametersClient parameter_client(
      node_, "/vision/shelf_drop_tf_node");
  if (!parameter_client.wait_for_service(500ms)) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] parametros de /vision/shelf_drop_tf_node no "
                "disponibles; se conserva preferred_y_m de launch");
    return;
  }

  std::vector<rclcpp::Parameter> parameters;
  if (has_semantic_preference || has_hand_preference) {
    parameters.emplace_back("preferred_y_m", preferred_y);
  }
  parameters.emplace_back("preferred_section_rank",
                          space_target.shelf_level_rank);

  auto future = parameter_client.set_parameters(parameters);
  if (future.wait_for(1s) != std::future_status::ready) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] timeout ajustando preferencias para shelf; se "
                "conserva el valor actual");
    return;
  }

  const auto results = future.get();
  if (results.empty() || !results.front().successful) {
    const std::string reason =
        results.empty() ? "empty_result" : results.front().reason;
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] fallo ajustando preferencias para shelf: %s",
                reason.c_str());
    return;
  }

  if (has_semantic_preference) {
    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] preferencias shelf por categoria: "
                "preferred_y_m=%.2f level_rank=%d %s",
                preferred_y, space_target.shelf_level_rank,
                semantic_detail.c_str());
  } else if (has_hand_preference) {
    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] preferencias shelf: preferred_y_m=%.2f "
                "level_rank=%d (left='%s' right='%s')",
                preferred_y, space_target.shelf_level_rank,
                left_object.get<std::string>().c_str(),
                right_object.get<std::string>().c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] preferencias shelf: preferred_y_m=launch "
                "level_rank=%d",
                space_target.shelf_level_rank);
  }
}

//category section
std::optional<std::string> FindSpace::heldObjectName() const {
  const json right_object =
      knowledge_->get_world_state("robot", "", "right_hand");
  const json left_object = knowledge_->get_world_state("robot", "", "left_hand");

  const auto resolve_held = [this](const json &held)
      -> std::optional<std::string> {
    if (!held.is_string()) {
      return std::nullopt;
    }

    const std::string value = held.get<std::string>();
    if (value.empty() || value == "nothing") {
      return std::nullopt;
    }

    if (auto object_name = objectNameFromId(value)) {
      return object_name;
    }

    return normalizeObjectToken(value);
  };

  if (auto right = resolve_held(right_object)) {
    return right;
  }

  return resolve_held(left_object);
}

std::optional<std::string>
FindSpace::objectNameFromId(const std::string &object_id) const {
  const std::size_t sep = object_id.rfind('_');
  if (sep == std::string::npos || sep == 0 || sep + 1 >= object_id.size()) {
    return std::nullopt;
  }

  const std::string category = object_id.substr(0, sep);
  const std::string index = object_id.substr(sep + 1);
  const json object_name = knowledge_->get_world_state(category, index, "name");
  if (!object_name.is_string()) {
    return std::nullopt;
  }

  const std::string normalized = normalizeObjectToken(object_name.get<std::string>());
  if (normalized.empty()) {
    return std::nullopt;
  }

  return normalized;
}

std::unordered_map<std::string, std::vector<std::string>>
FindSpace::loadObjectCategories() const {
  std::vector<std::string> candidate_paths;
  try {
    candidate_paths.push_back(
        ament_index_cpp::get_package_share_directory("knowledge_bank") +
        "/domains/objects.yaml");
  } catch (const std::exception &ex) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] knowledge_bank no encontrado por ament_index: %s",
                ex.what());
  }
  candidate_paths.push_back("src/Knowledge/knowledge_bank/domains/objects.yaml");

  std::ifstream file;
  std::string selected_path;
  for (const auto &path : candidate_paths) {
    file.open(path);
    if (file.is_open()) {
      selected_path = path;
      break;
    }
    file.clear();
  }

  std::unordered_map<std::string, std::vector<std::string>> categories;
  if (!file.is_open()) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] no se pudo abrir diccionario de objetos para "
                "preferencia semantica shelf");
    return categories;
  }

  std::string current_category;
  std::string line;
  while (std::getline(file, line)) {
    const std::size_t comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }

    const auto first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      continue;
    }

    const std::string trimmed = line.substr(first);
    if (first == 0 && trimmed.back() == ':') {
      current_category =
          normalizeObjectToken(trimmed.substr(0, trimmed.size() - 1));
      categories[current_category];
      continue;
    }

    if (trimmed.rfind("- ", 0) == 0 && !current_category.empty()) {
      std::string item = normalizeObjectToken(trimmed.substr(2));
      if (!item.empty()) {
        categories[current_category].push_back(item);
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(),
              "[FindSpace] diccionario shelf cargado desde %s categorias=%zu",
              selected_path.c_str(), categories.size());
  return categories;
}

std::optional<std::string> FindSpace::categoryForObject(
    const std::string &object_name,
    const std::unordered_map<std::string, std::vector<std::string>>
        &categories) const {
  const std::string normalized = normalizeObjectToken(object_name);
  for (const auto &[category, objects] : categories) {
    if (std::find(objects.begin(), objects.end(), normalized) != objects.end()) {
      return category;
    }
  }
  return std::nullopt;
}

std::optional<double> FindSpace::preferredShelfYFromCategory(
    const std::string &held_category,
    const std::vector<std::string> &category_objects) {
  if (!owl_v2_client_->wait_for_service(1s)) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] OWLv2 no disponible para preferencia semantica "
                "de shelf");
    return std::nullopt;
  }

  std::vector<std::string> queries;
  queries.reserve(category_objects.size());
  for (const auto &object : category_objects) {
    if (!object.empty()) {
      queries.push_back(object);
    }
  }

  if (queries.empty()) {
    return std::nullopt;
  }

  auto request =
      std::make_shared<vision_interfaces::srv::GetDetections::Request>();
  request->queries = queries;

  auto future = owl_v2_client_->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] timeout OWLv2 para preferencia semantica shelf");
    return std::nullopt;
  }

  const auto response = future.get();
  if (!response || !response->success || response->detections.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] OWLv2 sin detecciones para categoria shelf '%s'",
                held_category.c_str());
    return std::nullopt;
  }

  std::unordered_map<std::string, int> class_counts;
  double weighted_y_sum = 0.0;
  double weight_sum = 0.0;
  int usable_detections = 0;
  int far_detections = 0;
  constexpr double kMaxCategoryObjectDistXY = 1.30;
  constexpr double kMaxCategoryObjectAbsY = 0.75;

  for (const auto &det : response->detections) {
    const std::string normalized_class = normalizeObjectToken(det.class_name);
    if (std::find(category_objects.begin(), category_objects.end(),
                  normalized_class) == category_objects.end()) {
      continue;
    }

    std::string sanitized = det.class_name;
    std::replace(sanitized.begin(), sanitized.end(), ' ', '_');
    const int instance = class_counts[sanitized]++;
    const std::string frame = owlFrameName(det.class_name, instance);

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("base_link", frame, tf2::TimePointZero,
                                       300ms);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "[FindSpace] objeto shelf categoria=%s sin TF frame=%s "
                   "error=%s",
                   held_category.c_str(), frame.c_str(), ex.what());
      continue;
    }

    const double y = tf.transform.translation.y;
    const double x = tf.transform.translation.x;
    const double distance_xy = std::hypot(x, y);
    if (distance_xy > kMaxCategoryObjectDistXY ||
        std::fabs(y) > kMaxCategoryObjectAbsY) {
      ++far_detections;
      RCLCPP_DEBUG(node_->get_logger(),
                   "[FindSpace] objeto shelf categoria=%s ignorado por lejano "
                   "frame=%s x=%.3f y=%.3f dist_xy=%.3f",
                   held_category.c_str(), frame.c_str(), x, y, distance_xy);
      continue;
    }

    if (std::fabs(y) < 0.04) {
      continue;
    }

    const double confidence = std::max(0.05, det.confidence);
    weighted_y_sum += y * confidence;
    weight_sum += confidence;
    ++usable_detections;
    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] objeto shelf categoria=%s detectado=%s frame=%s "
                "x=%.3f y=%.3f dist_xy=%.3f conf=%.2f",
                held_category.c_str(), normalized_class.c_str(), frame.c_str(),
                x, y, distance_xy, det.confidence);
  }

  if (usable_detections == 0 || weight_sum <= 0.0) {
    if (far_detections > 0) {
      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] categoria shelf '%s' detectada solo lejos "
                  "(ignoradas=%d dist_xy_max=%.2f abs_y_max=%.2f)",
                  held_category.c_str(), far_detections,
                  kMaxCategoryObjectDistXY, kMaxCategoryObjectAbsY);
    }
    return std::nullopt;
  }

  const double mean_y = weighted_y_sum / weight_sum;
  constexpr double kPreferredShelfY = 0.18;
  const double preferred_y = mean_y >= 0.0 ? kPreferredShelfY
                                           : -kPreferredShelfY;

  RCLCPP_INFO(node_->get_logger(),
              "[FindSpace] categoria shelf '%s' ubicada en lado %s "
              "(mean_y=%.3f detecciones=%d ignoradas_lejos=%d) "
              "-> preferred_y_m=%.2f",
              held_category.c_str(), preferred_y >= 0.0 ? "izquierdo(+Y)"
                                                        : "derecho(-Y)",
              mean_y, usable_detections, far_detections, preferred_y);
  return preferred_y;
}

bool FindSpace::preScanShelf(const std::shared_ptr<GoalHandle> &gh,
                             const SpaceTarget &space_target) {
  struct ScanStep {
    double pan;
    double tilt;
    double vx;
    double vy;
    double wz;
    int pulse_ms;
    const char *label;
  };

  const std::array<ScanStep, 3> steps = {{
      {0.0, space_target.head_pitch, 0.0, 0.0, 0.0, 0, "centro"},
      {0.0, -0.35, 0.0, 0.0, 0.0, 0, "arriba"},
      {0.0, -0.75, 0.0, 0.0, 0.0, 0, "abajo"},
  }};

  int step_index = 0;
  for (const auto &step : steps) {
    if (!gh->is_active()) {
      return false;
    }
    publishFeedback(
        gh, FindSpaceState::PREPARE_SENSOR,
        "pre-scan shelf " + std::string(step.label) + " paso=" +
            std::to_string(++step_index));
    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] pre-scan shelf paso=%d/%zu %s head=(%.3f %.3f) "
                "base=(%.3f %.3f %.3f) duracion=%.2fs",
                step_index, steps.size(), step.label, step.pan, step.tilt,
                step.vx, step.vy, step.wz,
                static_cast<double>(step.pulse_ms) / 1000.0);

    if (!hardware_->MoveHead({step.pan, step.tilt})) {
      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] fallo al mover cabeza durante pre-scan shelf");
      return false;
    }
    if (step.pulse_ms > 0) {
      publishBasePulse(step.vx, step.vy, step.wz,
                       std::chrono::milliseconds(step.pulse_ms));
    }
    std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(
                space_target.sensor_settle_delay_sec)));
  }

  if (!hardware_->MoveHead({0.0, space_target.head_pitch})) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] fallo al regresar cabeza a postura de shelf");
    return false;
  }
  std::this_thread::sleep_for(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(
              space_target.sensor_settle_delay_sec)));
  return true;
}

bool FindSpace::waitForFreshShelfTf(
    const std::shared_ptr<GoalHandle> &gh,
    const SpaceTarget &space_target,
    const rclcpp::Time &request_stamp,
    std::chrono::milliseconds max_wait,
    geometry_msgs::msg::TransformStamped &tf,
    std::string &last_error) {
  const std::string target_frame = "base_link";
  const auto start_time = std::chrono::steady_clock::now();
  int attempts = 0;

  while (gh->is_active() &&
         std::chrono::steady_clock::now() - start_time < max_wait) {
    ++attempts;
    try {
      tf = tf_buffer_->lookupTransform(target_frame, space_target.frame,
                                       tf2::TimePointZero, 500ms);
      const rclcpp::Time tf_stamp(tf.header.stamp);
      if (tf_stamp.nanoseconds() != 0 && tf_stamp < request_stamp) {
        last_error = "stale_shelf_tf stamp=" +
                     std::to_string(tf_stamp.seconds()) +
                     " request_stamp=" +
                     std::to_string(request_stamp.seconds());
        std::this_thread::sleep_for(100ms);
        continue;
      }

      const double x = tf.transform.translation.x;
      const double y = tf.transform.translation.y;
      const double z = tf.transform.translation.z;
      const double distance_xy = std::hypot(x, y);
      const auto &b = space_target.bounds;
      if (x < b.x_min || x > b.x_max || std::fabs(y) > b.abs_y_max ||
          z < b.z_min || z > b.z_max || distance_xy > b.dist_xy_max) {
        last_error = "aligned_shelf_tf_out_of_bounds x=" +
                     std::to_string(x) + " y=" + std::to_string(y) +
                     " z=" + std::to_string(z) +
                     " dist_xy=" + std::to_string(distance_xy);
        std::this_thread::sleep_for(100ms);
        continue;
      }

      RCLCPP_INFO(node_->get_logger(),
                  "[FindSpace] shelf TF fresca tras alineacion intento=%d "
                  "base=(%.3f %.3f %.3f)",
                  attempts, x, y, z);
      return true;
    } catch (const tf2::TransformException &ex) {
      last_error = ex.what();
      std::this_thread::sleep_for(100ms);
    }
  }

  return false;
}

bool FindSpace::refineShelfAlignment(
    const std::shared_ptr<GoalHandle> &gh,
    const SpaceTarget &space_target,
    const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr &enable_client,
    geometry_msgs::msg::TransformStamped &tf) {
  constexpr int kMaxAlignmentPasses = 2;
  constexpr double kDesiredDropX = 0.60;  // Robot aprox. a 50 cm del estante.
  constexpr double kDesiredShelfYaw = -M_PI_2;
  constexpr double kXDeadband = 0.07;
  constexpr double kYDeadband = 0.045;
  constexpr double kYawDeadband = 0.07;
  constexpr double kMaxVx = 0.055;
  constexpr double kMaxVy = 0.045;
  constexpr double kMaxWz = 0.12;

  for (int pass = 1; pass <= kMaxAlignmentPasses; ++pass) {
    geometry_msgs::msg::TransformStamped shelf_tf;
    try {
      shelf_tf = tf_buffer_->lookupTransform("base_link", "shelf_frame",
                                             tf2::TimePointZero, 300ms);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] no se pudo leer shelf_frame para alineacion: %s; "
                  "se conserva TF shelf actual",
                  ex.what());
      return true;
    }

    const double drop_x = tf.transform.translation.x;
    const double drop_y = tf.transform.translation.y;
    const double shelf_yaw = yawFromTransform(shelf_tf);
    const double x_error = drop_x - kDesiredDropX;
    const double yaw_error = normalizeAngle(shelf_yaw - kDesiredShelfYaw);

    const bool x_ok = std::fabs(x_error) <= kXDeadband;
    const bool y_ok = std::fabs(drop_y) <= kYDeadband;
    const bool yaw_ok = std::fabs(yaw_error) <= kYawDeadband;

    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] shelf alignment pass=%d drop=(%.3f %.3f) "
                "x_error=%.3f shelf_yaw=%.3f yaw_error=%.3f ok=(x:%s y:%s yaw:%s)",
                pass, drop_x, drop_y, x_error, shelf_yaw, yaw_error,
                x_ok ? "true" : "false", y_ok ? "true" : "false",
                yaw_ok ? "true" : "false");

    if (x_ok && y_ok && yaw_ok) {
      return true;
    }

    const double vx = x_ok ? 0.0 : std::copysign(kMaxVx, x_error);
    const double vy = y_ok ? 0.0 : std::copysign(kMaxVy, drop_y);
    const double wz = yaw_ok ? 0.0
                             : std::clamp(yaw_error, -kMaxWz, kMaxWz);
    const double correction_size =
        std::max({std::fabs(x_error), std::fabs(drop_y),
                  0.45 * std::fabs(yaw_error)});
    const auto pulse_ms = static_cast<int>(
        std::clamp(350.0 + 1800.0 * correction_size, 350.0, 900.0));

    std::ostringstream detail;
    detail << "corrigiendo shelf pass=" << pass
           << " vx=" << vx << " vy=" << vy << " wz=" << wz
           << " duracion_ms=" << pulse_ms;
    publishFeedback(gh, FindSpaceState::PREPARE_SENSOR, detail.str());

    setSpaceTfNodeEnabled(enable_client, space_target.service_label, false);
    publishBasePulse(vx, vy, wz, std::chrono::milliseconds(pulse_ms));
    if (!hardware_->MoveHead({0.0, space_target.head_pitch})) {
      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] fallo al regresar cabeza tras alineacion shelf");
      return false;
    }
    std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(space_target.sensor_settle_delay_sec)));

    const rclcpp::Time request_stamp = node_->now();
    if (!setSpaceTfNodeEnabled(enable_client, space_target.service_label, true)) {
      return false;
    }

    std::string last_error;
    if (!waitForFreshShelfTf(gh, space_target, request_stamp, 7s, tf,
                             last_error)) {
      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] no se pudo recalcular shelf_drop_start tras "
                  "alineacion: %s",
                  last_error.c_str());
      return false;
    }
  }

  return true;
}
