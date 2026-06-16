#include "executor_tools/FindSpace.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nlohmann/json.hpp"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
FindSpace::FindSpace(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Hardware> &hardware,
           const std::shared_ptr<Knowledge> &knowledge)
    : node_(node), hardware_(hardware), knowledge_(knowledge) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  enable_table_drop_tf_client_ = node_->create_client<std_srvs::srv::SetBool>(
      "/vision/table_drop_tf/enable");
  enable_container_drop_tf_client_ = node_->create_client<std_srvs::srv::SetBool>(
          "/vision/container_drop_tf/enable");
  enable_shelf_drop_tf_client_ = node_->create_client<std_srvs::srv::SetBool>(
      "/vision/shelf_drop_tf/enable");
  owl_v2_client_ = node_->create_client<vision_interfaces::srv::GetDetections>(
      "/vision/owl/owlv2_service");
}

// =========================================================
// run
// =========================================================
Result FindSpace::run(const std::string &argument,
                 const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active()) {
    return {false, "FAILED", "goal_handle_inactive"};
  }

  // 1) Interpretamos el argumento del plan. En lugar de asumir que el
  // argumento es una TF, lo traducimos a un modo de percepcion:
  // mesa -> table_drop_start, contenedor -> container_drop_start.
  // estante/armario -> shelf_drop_start.
  const SpaceTarget space_target = resolveDropTarget(argument);
  const std::string space_frame = space_target.frame;
  const std::string target_frame = "base_link";
  auto enable_client = enable_table_drop_tf_client_;
  if (space_target.mode == "container") {
    enable_client = enable_container_drop_tf_client_;
  } else if (space_target.mode == "shelf") {
    enable_client = enable_shelf_drop_tf_client_;
  }

  std::ostringstream ready_detail;
  ready_detail << "arg=" << (argument.empty() ? "<empty>" : argument)
               << " mode=" << space_target.mode
               << " frame=" << space_target.frame;
  publishFeedback(gh, FindSpaceState::READY, ready_detail.str());
  RCLCPP_INFO(node_->get_logger(),
              "[FindSpace] objetivo: argumento='%s' modo=%s TF '%s' respecto a '%s'",
              argument.c_str(), space_target.mode.c_str(), space_frame.c_str(),
              target_frame.c_str());

  publishFeedback(gh, FindSpaceState::PREPARE_SENSOR,
                  "deteniendo base, ajustando torso y cabeza");
  hardware_->MoveBase(0.0, 0.0, 0.0);

  // 2) Preparamos la postura de sensores segun el objetivo. Para mesa la
  // cabeza mira menos hacia abajo; para contenedores en piso mira mas abajo.
  if (!hardware_->MoveTorso(space_target.torso_height)) {
    publishFeedback(gh, FindSpaceState::FAILED, "move_torso_failed");
    return {false, "FAILED", "move_torso_failed"};
  }

  if (!hardware_->MoveHead({0.0, space_target.head_pitch})) {
    publishFeedback(gh, FindSpaceState::FAILED, "move_head_failed");
    return {false, "FAILED", "move_head_failed"};
  }

  if (space_target.sensor_settle_delay_sec > 0.0) {
    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] esperando %.2fs para estabilizar cabeza/camara antes "
                "del escaneo %s",
                space_target.sensor_settle_delay_sec,
                space_target.mode.c_str());
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(space_target.sensor_settle_delay_sec)));
  }

  if (space_target.mode == "container" &&
      !precenterContainerWithOwl(gh, space_target)) {
    // Si OWLv2 ve el contenedor pero esta demasiado lejos/lateral, avisamos
    // temprano. Ese mensaje sera la pista para que un nodo de navegacion
    // reaproxime al robot antes de volver a llamar find_space.
    publishFeedback(gh, FindSpaceState::FAILED, "container_out_of_scan_zone");
    hardware_->MoveBase(0.0, 0.0, 0.0);
    return {false, "FAILED", "container_out_of_scan_zone"};
  }

  if (space_target.mode == "shelf" && !preScanShelf(gh, space_target)) {
    publishFeedback(gh, FindSpaceState::FAILED, "shelf_prescan_failed");
    hardware_->MoveBase(0.0, 0.0, 0.0);
    return {false, "FAILED", "shelf_prescan_failed"};
  }

  if (space_target.mode == "table") {
    configureTablePreferenceForHeldObject();
  } else if (space_target.mode == "shelf") {
    configureShelfPreference(space_target);
  }

  const rclcpp::Time request_stamp = node_->now();
  // 3) Activamos solo el nodo de vision necesario. Cada nodo procesa una nube
  // y publica una TF fresca cuando encuentra un punto seguro.
  if (!setSpaceTfNodeEnabled(enable_client, space_target.service_label, true)) {
    publishFeedback(gh, FindSpaceState::FAILED,
                    "enable_" + space_target.mode + "_drop_tf_failed");
    return {false, "FAILED",
            "enable_" + space_target.mode + "_drop_tf_failed"};
  }

  publishFeedback(gh, FindSpaceState::WAIT_SPACE_TF,
                  target_frame + " <- " + space_frame);

  const auto start_time = std::chrono::steady_clock::now();
  const auto max_wait = space_target.mode == "container" ? 32s : 8s;
  geometry_msgs::msg::TransformStamped tf;
  bool found = false;
  std::string last_error;
  int attempts = 0;
  std::shared_ptr<rclcpp::AsyncParametersClient> container_status_client;
  if (space_target.mode == "container") {
    container_status_client = std::make_shared<rclcpp::AsyncParametersClient>(
        node_, "/vision/container_drop_tf_node");
    if (!container_status_client->wait_for_service(300ms)) {
      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] parametros de /vision/container_drop_tf_node "
                  "no disponibles al iniciar espera; se seguira intentando");
    }
  }

  auto poseInBounds = [&space_target](
                          const geometry_msgs::msg::TransformStamped &candidate) {
    const double x = candidate.transform.translation.x;
    const double y = candidate.transform.translation.y;
    const double z = candidate.transform.translation.z;
    const double distance_xy = std::hypot(x, y);
    const auto &b = space_target.bounds;
    return x >= b.x_min && x <= b.x_max && std::fabs(y) <= b.abs_y_max &&
           z >= b.z_min && z <= b.z_max && distance_xy <= b.dist_xy_max;
  };

  auto readContainerDropStatus = [&container_status_client](
                                     std::string &result,
                                     std::string &error) {
    result.clear();
    error.clear();
    if (!container_status_client ||
        !container_status_client->service_is_ready()) {
      return false;
    }

    auto future = container_status_client->get_parameters(
        {"last_result", "last_error"});
    if (future.wait_for(60ms) != std::future_status::ready) {
      return false;
    }

    const auto params = future.get();
    if (!params.empty() &&
        params[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      result = params[0].as_string();
    }
    if (params.size() > 1 &&
        params[1].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      error = params[1].as_string();
    }
    return !result.empty();
  };

  // 4) Esperamos la TF fresca publicada por vision. Se ignoran TF viejas
  // anteriores a esta peticion y candidatos fuera de limites.
  // DEBUG: si aparece "ignorando TF vieja", el detector no genero una TF nueva
  // para este escaneo; revisar logs de container/table/shelf_drop_tf_node.
  while (gh->is_active() && std::chrono::steady_clock::now() - start_time < max_wait) {
    ++attempts;
    if (space_target.mode == "container") {
      std::string container_result;
      std::string container_error;
      if (readContainerDropStatus(container_result, container_error) &&
          container_result == "container_full") {
        last_error = container_error.empty() ? "container_full"
                                             : container_error;
        RCLCPP_WARN(node_->get_logger(),
                    "[FindSpace] contenedor lleno reportado por vision; "
                    "terminando espera de TF: %s",
                    last_error.c_str());
        publishFeedback(gh, FindSpaceState::FAILED,
                        "container_full: " + last_error);
        setSpaceTfNodeEnabled(enable_client, space_target.service_label, false);
        return {false, "FAILED", "container_full"};
      }
    }

    try {
      // Pedimos la transformacion mas reciente disponible. El nodo de vision
      // activo la publica como punto inicial seguro para soltar.
      tf = tf_buffer_->lookupTransform(target_frame, space_frame,
                                       tf2::TimePointZero, 500ms);
      const rclcpp::Time tf_stamp(tf.header.stamp);
      if (tf_stamp.nanoseconds() != 0 && tf_stamp < request_stamp) {
        last_error = "stale_drop_tf stamp=" + std::to_string(tf_stamp.seconds()) +
                     " request_stamp=" +
                     std::to_string(request_stamp.seconds());
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[FindSpace] ignorando TF vieja %s <- %s intento=%d %s",
                             target_frame.c_str(), space_frame.c_str(), attempts,
                             last_error.c_str());
        std::this_thread::sleep_for(100ms);
        continue;
      }
      if (!poseInBounds(tf)) {
        const double x = tf.transform.translation.x;
        const double y = tf.transform.translation.y;
        const double z = tf.transform.translation.z;
        const double distance_xy = std::hypot(x, y);
        last_error = "candidate_tf_out_of_bounds x=" + std::to_string(x) +
                     " y=" + std::to_string(y) +
                     " z=" + std::to_string(z) +
                     " dist_xy=" + std::to_string(distance_xy);
        RCLCPP_DEBUG(node_->get_logger(),
                     "[FindSpace] ignorando TF fuera de limites durante espera: %s",
                     last_error.c_str());
        std::this_thread::sleep_for(100ms);
        continue;
      }
      found = true;
      RCLCPP_INFO(node_->get_logger(),
                  "[FindSpace] TF encontrada tras %d intento(s) stamp=%.3f "
                  "request_stamp=%.3f",
                  attempts, tf_stamp.seconds(), request_stamp.seconds());
      break;
    } catch (const tf2::TransformException &ex) {
      last_error = ex.what();
      RCLCPP_DEBUG(node_->get_logger(),
                   "[FindSpace] esperando TF %s <- %s intento=%d error=%s",
                   target_frame.c_str(), space_frame.c_str(), attempts,
                   last_error.c_str());
      std::this_thread::sleep_for(100ms);
    }
  }

  if (!gh->is_active()) {
    setSpaceTfNodeEnabled(enable_client, space_target.service_label, false);
    publishFeedback(gh, FindSpaceState::FAILED, "goal_cancelled");
    return {false, "FAILED", "goal_cancelled"};
  }

  if (!found) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] no se encontro TF despues de %d intento(s). ultimo_error=%s",
                attempts, last_error.c_str());
    publishFeedback(gh, FindSpaceState::FAILED,
                    "find_space_tf_not_available: " + last_error);
    setSpaceTfNodeEnabled(enable_client, space_target.service_label, false);
    return {false, "FAILED", "find_space_tf_not_available"};
  }

  publishFeedback(gh, FindSpaceState::VALIDATE_SPACE_TF);

  const double x = tf.transform.translation.x;
  const double y = tf.transform.translation.y;
  const double z = tf.transform.translation.z;
  const double distance_xy = std::hypot(x, y);

  RCLCPP_INFO(node_->get_logger(),
              "[FindSpace] TF %s en %s: posicion=(%.3f, %.3f, %.3f) "
              "dist_xy=%.3f quat=(%.3f, %.3f, %.3f, %.3f)",
              space_frame.c_str(), target_frame.c_str(), x, y, z, distance_xy,
              tf.transform.rotation.x, tf.transform.rotation.y,
              tf.transform.rotation.z, tf.transform.rotation.w);

  const auto &bounds = space_target.bounds;
  // 5) Validacion final en base_link. Los limites son distintos para mesa y
  // contenedor porque la altura esperada cambia mucho.
  // DEBUG: si falla aqui, la TF existe pero no es una marca segura para usar
  // como objetivo de manipulacion; revisar la posicion base=(x,y,z) impresa.
  if (x < bounds.x_min || x > bounds.x_max ||
      std::fabs(y) > bounds.abs_y_max || z < bounds.z_min ||
      z > bounds.z_max || distance_xy > bounds.dist_xy_max) {
    std::ostringstream oss;
    oss << "pose_out_of_bounds x=" << x << " y=" << y << " z=" << z
        << " dist_xy=" << distance_xy
        << " limites x=[" << bounds.x_min << "," << bounds.x_max
        << "] abs_y<=" << bounds.abs_y_max << " z=[" << bounds.z_min
        << "," << bounds.z_max << "] dist<=" << bounds.dist_xy_max
        << " mode=" << space_target.mode;
    RCLCPP_WARN(node_->get_logger(), "[FindSpace] %s", oss.str().c_str());
    publishFeedback(gh, FindSpaceState::FAILED, oss.str());
    setSpaceTfNodeEnabled(enable_client, space_target.service_label, false);
    return {false, "FAILED", oss.str()};
  }

  std::ostringstream ok_msg;
  ok_msg << "find_space_tf_ready frame=" << space_frame << " base=(" << x
         << ", " << y << ", " << z << ") mode=" << space_target.mode;

  if (!storeSafeDropInKnowledge(space_target, tf)) {
    publishFeedback(gh, FindSpaceState::VALIDATE_SPACE_TF,
                    "safe_drop_knowledge_update_failed");
  }

  publishFeedback(gh, FindSpaceState::SUCCESS, ok_msg.str());
  setSpaceTfNodeEnabled(enable_client, space_target.service_label, false);
  return {true, "SUCCESS", ok_msg.str()};
}

// =========================================================
// setSpaceTfNodeEnabled
// =========================================================
bool FindSpace::setSpaceTfNodeEnabled(
    const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr &client,
    const std::string &service_label, bool enabled) {
  if (!client->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[FindSpace] servicio %s no disponible", service_label.c_str());
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;

  auto future = client->async_send_request(request);
  if (future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[FindSpace] timeout al %s %s", enabled ? "activar" : "desactivar",
                 service_label.c_str());
    return false;
  }

  const auto response = future.get();
  RCLCPP_INFO(node_->get_logger(), "[FindSpace] %s %s: %s",
              service_label.c_str(), enabled ? "enable" : "disable",
              response->message.c_str());
  return response->success;
}

void FindSpace::configureTablePreferenceForHeldObject() {
  const json left_object = knowledge_->get_world_state("robot", "", "left_hand");
  const json right_object =
      knowledge_->get_world_state("robot", "", "right_hand");

  if (!left_object.is_string() || !right_object.is_string()) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] no se pudo leer ocupacion de manos; se conserva "
                "preferred_x_m de launch");
    return;
  }

  const bool left_occupied = left_object.get<std::string>() != "nothing";
  const bool right_occupied = right_object.get<std::string>() != "nothing";

  if (!left_occupied && !right_occupied) {
    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] ninguna mano ocupada; se conserva preferred_x_m "
                "de launch para mesa");
    return;
  }

  const bool prefer_right = right_occupied || !left_occupied;
  const double preferred_x = prefer_right ? 0.35 : -0.35;
  const std::string preferred_hand = prefer_right ? "right_hand" : "left_hand";

  rclcpp::AsyncParametersClient parameter_client(
      node_, "/vision/table_drop_tf_node");
  if (!parameter_client.wait_for_service(500ms)) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] parametros de /vision/table_drop_tf_node no "
                "disponibles; se conserva preferred_x_m de launch");
    return;
  }

  auto future = parameter_client.set_parameters(
      {rclcpp::Parameter("preferred_x_m", preferred_x)});
  if (future.wait_for(1s) != std::future_status::ready) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] timeout ajustando preferred_x_m para mesa; se "
                "conserva el valor actual");
    return;
  }

  const auto results = future.get();
  if (results.empty() || !results.front().successful) {
    const std::string reason =
        results.empty() ? "empty_result" : results.front().reason;
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] fallo ajustando preferred_x_m para mesa: %s",
                reason.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[FindSpace] preferred_x_m de mesa ajustado a %.2f por %s "
              "ocupada (left='%s' right='%s')",
              preferred_x, preferred_hand.c_str(),
              left_object.get<std::string>().c_str(),
              right_object.get<std::string>().c_str());
}

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
    if (std::fabs(y) < 0.04) {
      continue;
    }

    const double confidence = std::max(0.05, det.confidence);
    weighted_y_sum += y * confidence;
    weight_sum += confidence;
    ++usable_detections;
    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] objeto shelf categoria=%s detectado=%s frame=%s "
                "y=%.3f conf=%.2f",
                held_category.c_str(), normalized_class.c_str(), frame.c_str(),
                y, det.confidence);
  }

  if (usable_detections == 0 || weight_sum <= 0.0) {
    return std::nullopt;
  }

  const double mean_y = weighted_y_sum / weight_sum;
  constexpr double kPreferredShelfY = 0.18;
  const double preferred_y = mean_y >= 0.0 ? kPreferredShelfY
                                           : -kPreferredShelfY;

  RCLCPP_INFO(node_->get_logger(),
              "[FindSpace] categoria shelf '%s' ubicada en lado %s "
              "(mean_y=%.3f detecciones=%d) -> preferred_y_m=%.2f",
              held_category.c_str(), preferred_y >= 0.0 ? "izquierdo(+Y)"
                                                        : "derecho(-Y)",
              mean_y, usable_detections, preferred_y);
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

  const std::array<ScanStep, 8> steps = {{
      {0.0, space_target.head_pitch, 0.0, 0.0, 0.0, 0, "centro"},
      {0.0, -0.35, 0.0, 0.0, 0.0, 0, "arriba"},
      {0.0, -0.75, 0.0, 0.0, 0.0, 0, "abajo"},
      {0.30, space_target.head_pitch, 0.0, 0.0, 0.0, 0, "izquierda"},
      {-0.30, space_target.head_pitch, 0.0, 0.0, 0.0, 0, "derecha"},
      {0.0, space_target.head_pitch, 0.0, 0.0, 0.14, 450, "giro-izquierda"},
      {0.0, space_target.head_pitch, 0.0, 0.0, -0.28, 850, "giro-derecha"},
      {0.0, space_target.head_pitch, 0.0, 0.0, 0.14, 450, "regresa-orientacion"},
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

bool FindSpace::precenterContainerWithOwl(
    const std::shared_ptr<GoalHandle> &gh,
    const SpaceTarget &space_target) {
  constexpr int kMaxHeadCorrections = 4;
  constexpr int kMaxBaseCorrections = 7;
  constexpr double kIdealXMin = 0.75;
  constexpr double kIdealXMax = 0.85;
  constexpr double kIdealAbsYMax = 0.22;
  constexpr double kScanXMin = 0.65;
  constexpr double kScanXMax = 1.60;
  constexpr double kScanAbsYMax = 0.65;
  constexpr double kActionXMin = 0.45;
  constexpr double kActionXMax = 2.00;
  constexpr double kActionAbsYMax = 1.10;
  constexpr double kImageCenterX = 320.0;
  constexpr double kImageCenterTolerancePx = 55.0;
  constexpr double kHeadPanMax = 0.55;
  constexpr double kHeadPanMin = -0.55;
  constexpr double kHeadTiltMin = -0.95;
  constexpr double kHeadTiltMax = -0.35;
  constexpr double kMaxVx = 0.10;
  constexpr double kMaxVy = 0.07;
  constexpr double kMaxWz = 0.18;

  struct SearchStep {
    double pan;
    double tilt_offset;
    double vx;
    double vy;
    double wz;
    int pulse_ms;
    const char *label;
  };

  const std::array<SearchStep, 14> search_steps = {{
      {0.0, 0.0, 0.0, 0.0, 0.0, 0, "centro"},
      {-0.35, 0.0, 0.0, 0.0, 0.0, 0, "derecha"},
      {0.35, 0.0, 0.0, 0.0, 0.0, 0, "izquierda"},
      {0.0, -0.14, 0.0, 0.0, 0.0, 0, "abajo"},
      {-0.35, -0.14, 0.0, 0.0, 0.0, 0, "abajo-derecha"},
      {0.35, -0.14, 0.0, 0.0, 0.0, 0, "abajo-izquierda"},
      {0.0, 0.16, 0.0, 0.0, 0.0, 0, "arriba"},
      {-0.35, 0.16, 0.0, 0.0, 0.0, 0, "arriba-derecha"},
      {0.35, 0.16, 0.0, 0.0, 0.0, 0, "arriba-izquierda"},
      {0.0, 0.0, 0.0, 0.0, kMaxWz, 650, "giro-izquierda"},
      {0.0, 0.0, 0.0, 0.0, -2.0 * kMaxWz, 1200, "giro-derecha"},
      {0.0, 0.0, 0.0, 0.0, kMaxWz, 650, "regresa-orientacion"},
      {0.0, 0.0, kMaxVx * 0.8, 0.0, 0.0, 650, "adelante"},
      {0.0, 0.0, -kMaxVx * 0.7, 0.0, 0.0, 600, "atras"},
  }};

  double commanded_pan = 0.0;
  double commanded_tilt = space_target.head_pitch;
  int head_corrections = 0;
  int base_corrections = 0;
  int search_step_index = 0;
  // Esta etapa no decide el punto de drop. Solo intenta poner al robot/camara
  // en una postura donde el detector geometrico vea suficiente nube util.
  for (int attempt = 0;
       attempt <= kMaxHeadCorrections + kMaxBaseCorrections +
                      static_cast<int>(search_steps.size()) + 1;
       ++attempt) {
    if (!gh->is_active()) {
      return false;
    }

    publishFeedback(
        gh, FindSpaceState::PRECENTER_CONTAINER_WITH_OWL,
        "centrando contenedor con cabeza/base y OWLv2 intento=" +
            std::to_string(attempt + 1));

    OwlContainerPose pose;
    std::string error;
    if (!requestOwlContainerPose(pose, error)) {
      if (search_step_index < static_cast<int>(search_steps.size())) {
        const auto &step = search_steps[search_step_index++];
        commanded_pan = step.pan;
        commanded_tilt =
            std::clamp(space_target.head_pitch + step.tilt_offset,
                       kHeadTiltMin, kHeadTiltMax);
        RCLCPP_WARN(node_->get_logger(),
                    "[FindSpace] precenter OWL sin deteccion (%s); "
                    "busqueda activa paso=%d/%zu %s head=(%.3f %.3f) "
                    "base=(%.3f %.3f %.3f) duracion=%.2fs",
                    error.c_str(), search_step_index, search_steps.size(),
                    step.label, commanded_pan, commanded_tilt, step.vx,
                    step.vy, step.wz,
                    static_cast<double>(step.pulse_ms) / 1000.0);
        if (!hardware_->MoveHead({commanded_pan, commanded_tilt})) {
          RCLCPP_WARN(node_->get_logger(),
                      "[FindSpace] fallo al mover cabeza en busqueda activa");
        }
        if (step.pulse_ms > 0) {
          publishBasePulse(step.vx, step.vy, step.wz,
                           std::chrono::milliseconds(step.pulse_ms));
        }
        std::this_thread::sleep_for(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(
                    space_target.sensor_settle_delay_sec)));
        continue;
      }

      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] precenter OWL no disponible/sin deteccion tras "
                  "busqueda activa: %s; se usara fallback geometrico",
                  error.c_str());
      return true;
    }

    const bool in_scan_zone =
        pose.x >= kScanXMin && pose.x <= kScanXMax &&
        std::fabs(pose.y) <= kScanAbsYMax;
    const bool in_ideal_zone =
        pose.x >= kIdealXMin && pose.x <= kIdealXMax &&
        std::fabs(pose.y) <= kIdealAbsYMax;
    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] precenter OWL class=%s frame=%s conf=%.2f "
                "base=(%.3f %.3f %.3f) bbox_cx=%.1f bbox_size=(%.1f %.1f) "
                "ideal_x=[%.2f %.2f] ideal_abs_y<=%.2f "
                "scan_x=[%.2f %.2f] scan_abs_y<=%.2f",
                pose.class_name.c_str(), pose.frame.c_str(), pose.confidence,
                pose.x, pose.y, pose.z, pose.bbox_center_x, pose.bbox_width,
                pose.bbox_height, kIdealXMin, kIdealXMax, kIdealAbsYMax,
                kScanXMin, kScanXMax, kScanAbsYMax);
    // DEBUG: este log es la referencia principal para saber si el problema es
    // de vista (bbox_cx), distancia (x) o lateralidad (y).

    if (!in_scan_zone) {
      const bool in_action_zone =
          pose.x >= kActionXMin && pose.x <= kActionXMax &&
          std::fabs(pose.y) <= kActionAbsYMax;
      if (in_action_zone && base_corrections < kMaxBaseCorrections) {
        double vx = 0.0;
        double vy = 0.0;
        double wz = 0.0;
        if (pose.x > kIdealXMax) {
          vx = kMaxVx;
        } else if (pose.x < kIdealXMin) {
          vx = -0.8 * kMaxVx;
        }
        if (pose.y > kIdealAbsYMax) {
          vy = kMaxVy;
          wz = kMaxWz;
        } else if (pose.y < -kIdealAbsYMax) {
          vy = -kMaxVy;
          wz = -kMaxWz;
        }

        const double x_error = std::max(
            {0.0, pose.x - kIdealXMax, kIdealXMin - pose.x});
        const double y_error =
            std::max(0.0, std::fabs(pose.y) - kIdealAbsYMax);
        const auto pulse_ms = static_cast<int>(std::clamp(
            500.0 + 850.0 * std::max(x_error, y_error), 500.0, 1100.0));

        ++base_corrections;
        RCLCPP_INFO(node_->get_logger(),
                    "[FindSpace] contenedor fuera de zona de escaneo pero "
                    "corregible; base vx=%.3f vy=%.3f wz=%.3f duracion=%.2fs "
                    "base_correction=%d/%d",
                    vx, vy, wz, static_cast<double>(pulse_ms) / 1000.0,
                    base_corrections, kMaxBaseCorrections);
        publishBasePulse(vx, vy, wz, std::chrono::milliseconds(pulse_ms));
        commanded_pan = 0.0;
        commanded_tilt = space_target.head_pitch;
        hardware_->MoveHead({commanded_pan, commanded_tilt});
        std::this_thread::sleep_for(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(
                    space_target.sensor_settle_delay_sec)));
        continue;
      }

      RCLCPP_WARN(node_->get_logger(),
                  "[FindSpace] contenedor fuera de la zona corregible actual: "
                  "base=(%.3f %.3f %.3f) scan_x=[%.2f %.2f] abs_y<=%.2f "
                  "action_x=[%.2f %.2f] action_abs_y<=%.2f. "
                  "Se requiere reaproximacion externa.",
                  pose.x, pose.y, pose.z, kScanXMin, kScanXMax,
                  kScanAbsYMax, kActionXMin, kActionXMax, kActionAbsYMax);
      return false;
    }

    const double image_error =
        pose.bbox_center_x > 0.0 ? pose.bbox_center_x - kImageCenterX : 0.0;
    const double desired_pan =
        std::clamp(std::atan2(pose.y, std::max(0.05, pose.x)), kHeadPanMin,
                   kHeadPanMax);
    const bool image_centered =
        std::fabs(image_error) <= kImageCenterTolerancePx;
    const bool pan_close = std::fabs(desired_pan - commanded_pan) <= 0.05;
    if (in_ideal_zone && image_centered && pan_close) {
      RCLCPP_INFO(node_->get_logger(),
                  "[FindSpace] precenter OWL listo: contenedor en zona ideal "
                  "para procesar nube");
      return true;
    }

    if ((!image_centered || !pan_close) &&
        head_corrections < kMaxHeadCorrections) {
      commanded_pan = desired_pan;
      commanded_tilt = space_target.head_pitch;
      ++head_corrections;
      RCLCPP_INFO(node_->get_logger(),
                  "[FindSpace] precenter moviendo cabeza pan=%.3f tilt=%.3f "
                  "image_error=%.1f head_correction=%d/%d",
                  commanded_pan, space_target.head_pitch, image_error,
                  head_corrections, kMaxHeadCorrections);
      if (!hardware_->MoveHead({commanded_pan, commanded_tilt})) {
        RCLCPP_WARN(node_->get_logger(),
                    "[FindSpace] fallo al mover cabeza para centrar contenedor; "
                    "se intentara continuar con geometria/base si aplica");
      }
      std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::duration<double>(space_target.sensor_settle_delay_sec)));
      continue;
    }

    if ((!image_centered || !pan_close) &&
        base_corrections < kMaxBaseCorrections) {
      const double pan_error = desired_pan - commanded_pan;
      double wz = 0.0;
      if (std::fabs(pan_error) > 0.03) {
        wz = std::clamp(pan_error, -kMaxWz, kMaxWz);
      } else if (pose.bbox_center_x > 0.0) {
        wz = image_error > 0.0 ? -kMaxWz : kMaxWz;
      }
      if (std::fabs(wz) > 0.001) {
        ++base_corrections;
        RCLCPP_INFO(node_->get_logger(),
                    "[FindSpace] precenter girando base para centrar vista "
                    "wz=%.3f image_error=%.1f desired_pan=%.3f "
                    "base_correction=%d/%d",
                    wz, image_error, desired_pan, base_corrections,
                    kMaxBaseCorrections);
        publishBasePulse(0.0, 0.0, wz, 550ms);
        commanded_pan = 0.0;
        commanded_tilt = space_target.head_pitch;
        hardware_->MoveHead({commanded_pan, commanded_tilt});
        std::this_thread::sleep_for(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(
                    space_target.sensor_settle_delay_sec)));
        continue;
      }
    }

    if (!in_ideal_zone && base_corrections < kMaxBaseCorrections) {
      double vx = 0.0;
      double vy = 0.0;
      double wz = 0.0;
      if (pose.x > kIdealXMax) {
        vx = kMaxVx;
      } else if (pose.x < kIdealXMin) {
        vx = -kMaxVx;
      }
      if (pose.y > kIdealAbsYMax) {
        vy = kMaxVy;
        if (std::fabs(pose.y) > 0.35) {
          wz = kMaxWz;
        }
      } else if (pose.y < -kIdealAbsYMax) {
        vy = -kMaxVy;
        if (std::fabs(pose.y) > 0.35) {
          wz = -kMaxWz;
        }
      }

      const double x_error = std::max(
          {0.0, pose.x - kIdealXMax, kIdealXMin - pose.x});
      const double y_error =
          std::max(0.0, std::fabs(pose.y) - kIdealAbsYMax);
      const auto pulse_ms = static_cast<int>(std::clamp(
          450.0 + 900.0 * std::max(x_error, y_error), 450.0, 950.0));

      ++base_corrections;
      RCLCPP_INFO(node_->get_logger(),
                  "[FindSpace] precenter corrigiendo base vx=%.3f vy=%.3f "
                  "wz=%.3f duracion=%.2fs base_correction=%d/%d",
                  vx, vy, wz, static_cast<double>(pulse_ms) / 1000.0,
                  base_corrections, kMaxBaseCorrections);
      publishBasePulse(vx, vy, wz, std::chrono::milliseconds(pulse_ms));
      commanded_pan = 0.0;
      commanded_tilt = space_target.head_pitch;
      hardware_->MoveHead({commanded_pan, commanded_tilt});
      std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::duration<double>(space_target.sensor_settle_delay_sec)));
      continue;
    }

    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] no se logro zona ideal tras correcciones; "
                "se continuara con geometria si sigue en zona escaneable");
    return true;
  }

  return true;
}

bool FindSpace::requestOwlContainerPose(OwlContainerPose &pose_out,
                                   std::string &error_out) {
  // OWLv2 se usa aqui como apoyo de precentrado. La decision final de espacio
  // seguro sigue en la geometria de los nodos *_drop_tf_node.
  if (!owl_v2_client_->wait_for_service(1s)) {
    error_out = "owl_service_not_available";
    return false;
  }

  auto request =
      std::make_shared<vision_interfaces::srv::GetDetections::Request>();
  request->queries = {"container_drop"};

  auto future = owl_v2_client_->async_send_request(request);
  if (future.wait_for(4s) != std::future_status::ready) {
    error_out = "owl_service_timeout";
    return false;
  }

  const auto response = future.get();
  if (!response || !response->success || response->detections.empty()) {
    error_out = response ? response->message : "empty_owl_response";
    if (error_out.empty()) {
      error_out = "no_owl_detections";
    }
    return false;
  }

  constexpr double kIdealXMin = 0.75;
  constexpr double kIdealXMax = 0.85;
  constexpr double kIdealAbsYMax = 0.22;
  constexpr double kScanXMin = 0.65;
  constexpr double kScanXMax = 1.60;
  constexpr double kScanAbsYMax = 0.65;
  constexpr double kImageCenterX = 320.0;

  bool found = false;
  double best_score = -1.0e9;
  std::string best_frame;
  std::string best_class;
  double best_confidence = 0.0;
  double best_bbox_center_x = 0.0;
  double best_bbox_width = 0.0;
  double best_bbox_height = 0.0;
  geometry_msgs::msg::TransformStamped best_tf;
  std::unordered_map<std::string, int> class_counts;

  for (const auto &det : response->detections) {
    const double width = std::max(0.0, det.x_max - det.x_min);
    const double height = std::max(0.0, det.y_max - det.y_min);
    if (width < 15.0 || height < 15.0) {
      continue;
    }

    const std::string frame_class = det.class_name;
    std::string sanitized = frame_class;
    std::replace(sanitized.begin(), sanitized.end(), ' ', '_');
    const int instance = class_counts[sanitized]++;
    const double area = width * height;
    const double center_x = 0.5 * (det.x_min + det.x_max);
    const std::string frame = owlFrameName(frame_class, instance);

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("base_link", frame, tf2::TimePointZero,
                                       500ms);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "[FindSpace] OWL candidato sin TF frame=%s error=%s",
                   frame.c_str(), ex.what());
      continue;
    }

    const double x = tf.transform.translation.x;
    const double y = tf.transform.translation.y;
    const bool in_scan_zone =
        x >= kScanXMin && x <= kScanXMax && std::fabs(y) <= kScanAbsYMax;
    const double ideal_x_error =
        std::max({0.0, kIdealXMin - x, x - kIdealXMax});
    const double ideal_y_error =
        std::max(0.0, std::fabs(y) - kIdealAbsYMax);
    const double scan_x_error =
        std::max({0.0, kScanXMin - x, x - kScanXMax});
    const double scan_y_error =
        std::max(0.0, std::fabs(y) - kScanAbsYMax);
    const double center_offset = std::fabs(center_x - kImageCenterX) /
                                 kImageCenterX;
    const double area_score = area / 10000.0;
    const double distance_score =
        (x >= kScanXMin && x <= kScanXMax)
            ? (kScanXMax - x) / std::max(0.001, kScanXMax - kScanXMin)
            : -std::fabs(x - kIdealXMin);
    const double score =
        0.35 * det.confidence + 1.20 * area_score +
        1.00 * distance_score + (in_scan_zone ? 3.0 : 0.0) -
        4.0 * std::hypot(ideal_x_error, ideal_y_error) -
        8.0 * std::hypot(scan_x_error, scan_y_error) -
        0.20 * center_offset;

    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] OWL candidato frame=%s conf=%.2f "
                "base=(%.3f %.3f %.3f) bbox_cx=%.1f area=%.0f "
                "score=%.3f in_scan=%s",
                frame.c_str(), det.confidence, x, y, tf.transform.translation.z,
                center_x, area, score, in_scan_zone ? "true" : "false");

    if (!found || score > best_score) {
      found = true;
      best_score = score;
      best_frame = frame;
      best_class = frame_class;
      best_confidence = det.confidence;
      best_bbox_center_x = center_x;
      best_bbox_width = width;
      best_bbox_height = height;
      best_tf = tf;
    }
  }

  if (!found) {
    error_out = "owl_detections_without_valid_tf_or_too_small";
    return false;
  }

  pose_out.detected = true;
  pose_out.class_name = best_class;
  pose_out.frame = best_frame;
  pose_out.confidence = best_confidence;
  pose_out.bbox_center_x = best_bbox_center_x;
  pose_out.bbox_width = best_bbox_width;
  pose_out.bbox_height = best_bbox_height;
  pose_out.x = best_tf.transform.translation.x;
  pose_out.y = best_tf.transform.translation.y;
  pose_out.z = best_tf.transform.translation.z;
  return true;
}

void FindSpace::publishBasePulse(double vx, double vy, double wz,
                            std::chrono::milliseconds duration) {
  // Pulso corto y siempre con stop final. Esto evita que FindSpace se convierta
  // en navegacion completa: solo corrige errores pequenos antes del escaneo.
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < duration) {
    hardware_->MoveBase(vx, vy, wz);
    std::this_thread::sleep_for(50ms);
  }
  hardware_->MoveBase(0.0, 0.0, 0.0);
}

bool FindSpace::storeSafeDropInKnowledge(
    const SpaceTarget &space_target,
    const geometry_msgs::msg::TransformStamped &tf) {
  if (!knowledge_) {
    return false;
  }

  const nlohmann::json pose = nlohmann::json::array({
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w,
  });

  const bool mode_ok = knowledge_->update_world_state(
      "robot", "", "safe_drop_mode", space_target.mode);
  const bool frame_ok = knowledge_->update_world_state(
      "robot", "", "safe_drop_frame", space_target.frame);
  const bool pose_ok = knowledge_->update_world_state(
      "robot", "", "safe_drop_pose", pose.dump());

  if (!mode_ok || !frame_ok || !pose_ok) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] no se pudo guardar safe_drop en knowledge "
                "mode_ok=%d frame_ok=%d pose_ok=%d",
                mode_ok, frame_ok, pose_ok);
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[FindSpace] safe_drop guardado en knowledge: mode=%s frame=%s "
              "pose=%s",
              space_target.mode.c_str(), space_target.frame.c_str(),
              pose.dump().c_str());
  return true;
}

FindSpace::SpaceTarget FindSpace::resolveDropTarget(const std::string &argument) {
  // Mapeo conservador:
  // - left_table/table/mesa siguen usando la logica de mesa.
  // - trash/bin/box/container/bote/basura/caja usan la logica de contenedor.
  // - shelf/rack/cabinet/estante/armario/repisa usan la logica de estante.
  // - si no se reconoce nada, el default sigue siendo mesa para compatibilidad.
  std::string lower = argument;
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });

  const auto contains = [&lower](const std::string &needle) {
    return lower.find(needle) != std::string::npos;
  };

  SpaceTarget target;
  target.mode = "table";
  target.frame = "table_drop_start";
  target.service_label = "/vision/table_drop_tf/enable";
  target.torso_height = 0.3;
  target.head_pitch = -0.7;
  target.bounds = {0.15, 0.80, 0.55, 0.35, 1.40, 1.05};

  const bool explicit_container = contains("container_drop_start");
  const bool explicit_table = contains("table_drop_start");
  const bool explicit_shelf = contains("shelf_drop_start");
  const bool looks_like_container =
      contains("trash") || contains("bin") || contains("box") ||
      contains("container") || contains("bote") || contains("basura") ||
      contains("caja") || contains("contenedor") || contains("cubeta");
  const bool looks_like_table = contains("table") || contains("mesa");
  const bool looks_like_shelf =
      contains("shelf") || contains("rack") || contains("cabinet") ||
      contains("bookcase") || contains("estante") || contains("armario") ||
      contains("repisa");

  if (explicit_container || (looks_like_container && !looks_like_table)) {
    target.mode = "container";
    target.frame = "container_drop_start";
    target.service_label = "/vision/container_drop_tf/enable";
    target.torso_height = 0.1;
    target.head_pitch = -0.7;
    target.sensor_settle_delay_sec = 1.6;
    target.bounds = {0.15, 1.35, 0.45, 0.08, 1.10, 1.50};
  }

  if (explicit_shelf || (looks_like_shelf && !looks_like_container)) {
    target.mode = "shelf";
    target.frame = "shelf_drop_start";
    target.service_label = "/vision/shelf_drop_tf/enable";
    target.torso_height = 0.3;
    target.head_pitch = -0.45;
    target.sensor_settle_delay_sec = 0.8;
    target.bounds = {0.15, 1.15, 0.60, 0.30, 1.55, 1.30};
    if (contains("top") || contains("upper") || contains("alto") ||
        contains("arriba") || contains("superior") || contains("primer") ||
        contains("primero") || contains("level 1") || contains("level_1") ||
        contains("nivel 1") || contains("nivel_1") || contains(" 1")) {
      target.shelf_level_rank = 1;
      target.torso_height = 0.35;
      target.head_pitch = -0.35;
    } else if (contains("middle") || contains("mid") || contains("medio") ||
               contains("segundo") || contains("second") ||
               contains("level 2") || contains("level_2") ||
               contains("nivel 2") || contains("nivel_2") || contains(" 2")) {
      target.shelf_level_rank = 2;
    } else if (contains("bottom") || contains("lower") ||
               contains("abajo") || contains("bajo") ||
               contains("inferior") || contains("tercer") ||
               contains("tercero") || contains("third") ||
               contains("level 3") || contains("level_3") ||
               contains("nivel 3") || contains("nivel_3") || contains(" 3")) {
      target.shelf_level_rank = 3;
      target.torso_height = 0.25;
      target.head_pitch = -0.65;
    }
  }

  if (explicit_table) {
    target.mode = "table";
    target.frame = "table_drop_start";
    target.service_label = "/vision/table_drop_tf/enable";
    target.torso_height = 0.3;
    target.head_pitch = -0.;
    target.bounds = {0.15, 0.80, 0.55, 0.35, 1.40, 1.05};
  }

  return target;
}

std::string FindSpace::owlFrameName(const std::string &class_name, int instance) {
  std::string sanitized = class_name;
  std::replace(sanitized.begin(), sanitized.end(), ' ', '_');
  return "OWL_" + sanitized + "_" + std::to_string(instance);
}

//category 
std::string FindSpace::normalizeObjectToken(const std::string &name) {
  std::string normalized = name;
  const auto first = normalized.find_first_not_of(" \t\r\n\"'");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = normalized.find_last_not_of(" \t\r\n\"'");
  normalized = normalized.substr(first, last - first + 1);

  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c) {
                   if (std::isspace(c) || c == '-') {
                     return '_';
                   }
                   return static_cast<char>(std::tolower(c));
                 });
  return normalized;
}

// =========================================================
// publishFeedback
// =========================================================
void FindSpace::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                           FindSpaceState state, const std::string &detail) {
  if (!gh->is_active()) {
    return;
  }

  std::string msg = "[FindSpace] " + stateName(state);
  if (!detail.empty()) {
    msg += ": " + detail;
  }

  auto fb = std::make_shared<ExecuteTask::Feedback>();
  fb->feedback.message = msg;
  gh->publish_feedback(fb);

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string FindSpace::stateName(FindSpaceState s) {
  switch (s) {
  case FindSpaceState::READY:
    return "READY";
  case FindSpaceState::PREPARE_SENSOR:
    return "PREPARE_SENSOR";
  case FindSpaceState::PRECENTER_CONTAINER_WITH_OWL:
    return "PRECENTER_CONTAINER_WITH_OWL";
  case FindSpaceState::WAIT_SPACE_TF:
    return "WAIT_SPACE_TF";
  case FindSpaceState::VALIDATE_SPACE_TF:
    return "VALIDATE_SPACE_TF";
  case FindSpaceState::SUCCESS:
    return "SUCCESS";
  case FindSpaceState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}
