#include "executor_tools/FindSpace.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <future>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>

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
  if (space_target.mode == "container" &&
      !configureContainerDropProfile(space_target)) {
    publishFeedback(gh, FindSpaceState::FAILED,
                    "configure_container_drop_profile_failed");
    return {false, "FAILED", "configure_container_drop_profile_failed"};
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

  if (space_target.mode == "shelf" &&
      !refineShelfAlignment(gh, space_target, enable_client, tf)) {
    publishFeedback(gh, FindSpaceState::FAILED, "shelf_alignment_failed");
    setSpaceTfNodeEnabled(enable_client, space_target.service_label, false);
    return {false, "FAILED", "shelf_alignment_failed"};
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

bool FindSpace::configureContainerDropProfile(const SpaceTarget &space_target) {
  if (!space_target.configure_container_drop_z) {
    return true;
  }

  auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, "/vision/container_drop_tf_node");
  if (!parameter_client->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[FindSpace] parametros de /vision/container_drop_tf_node "
                 "no disponibles para configurar perfil de contenedor");
    return false;
  }

  const std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("override_drop_z", true),
      rclcpp::Parameter("drop_z_override_m",
                        space_target.container_drop_z_override_m),
      rclcpp::Parameter("safe_z_max_m", space_target.bounds.z_max),
      rclcpp::Parameter("roi_z_max_m", space_target.container_roi_z_max_m),
  };

  auto future = parameter_client->set_parameters(parameters);
  if (future.wait_for(1s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[FindSpace] timeout configurando perfil de contenedor "
                 "drop_z=%.3f safe_z_max=%.3f roi_z_max=%.3f",
                 space_target.container_drop_z_override_m,
                 space_target.bounds.z_max, space_target.container_roi_z_max_m);
    return false;
  }

  bool all_ok = true;
  std::string error;
  for (const auto &result : future.get()) {
    if (!result.successful) {
      all_ok = false;
      if (!error.empty()) {
        error += "; ";
      }
      error += result.reason;
    }
  }
  if (!all_ok) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[FindSpace] fallo configurando perfil de contenedor: %s",
                 error.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[FindSpace] perfil de contenedor configurado: "
              "drop_z_override=%.3f safe_z_max=%.3f roi_z_max=%.3f",
              space_target.container_drop_z_override_m,
              space_target.bounds.z_max, space_target.container_roi_z_max_m);
  return true;
}

FindSpace::SpaceTarget FindSpace::resolveDropTarget(const std::string &argument) {
  // Mapeo conservador:
  // - left_table/table/mesa siguen usando la logica de mesa.
  // - trash/bin/box/container/bote/basura/caja usan la logica de contenedor.
  // - container_top/container_down seleccionan la postura de busqueda del
  //   contenedor sin cambiar el contrato final con Drop: mode=container y
  //   frame=container_drop_start.
  // - shelf/rack/cabinet/estante/armario/repisa usan la logica de estante.
  // - si no se reconoce nada, el default sigue siendo mesa para compatibilidad.
  constexpr double kContainerDownTorsoHeight = 0.05;
  constexpr double kContainerDownHeadPitch = -0.7;
  constexpr double kContainerDownDropZOverride = 1.08;
  constexpr double kContainerDownSafeZMax = 1.10;
  constexpr double kContainerDownRoiZMax = 1.05;
  constexpr double kContainerTopTorsoHeight = 0.3;
  // Ajuste manual principal para contenedores sobre mesa/lugar medio alto.
  constexpr double kContainerTopHeadPitch = -0.45;
  constexpr double kContainerTopDropZOverride = 1.20;
  constexpr double kContainerTopSafeZMax = 1.30;
  constexpr double kContainerTopRoiZMax = 1.35;

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
  const bool container_top =
      contains("container_top") || contains("container_up") ||
      contains("container_high") || contains("container_table") ||
      contains("bote_alto") || contains("contenedor_alto") ||
      contains("sobre_mesa") || contains("mesa_alto") ||
      contains("medio_alto");
  const bool container_down =
      contains("container_down") || contains("container_floor") ||
      contains("container_low") || contains("bote_piso") ||
      contains("contenedor_piso") || contains("en_piso") ||
      contains("suelo");
  const bool looks_like_table = contains("table") || contains("mesa");
  const bool looks_like_shelf =
      contains("shelf") || contains("rack") || contains("cabinet") ||
      contains("bookcase") || contains("estante") || contains("armario") ||
      contains("repisa");

  if (explicit_container || container_top || container_down ||
      (looks_like_container && !looks_like_table)) {
    target.mode = "container";
    target.frame = "container_drop_start";
    target.service_label = "/vision/container_drop_tf/enable";
    target.torso_height = kContainerDownTorsoHeight;
    target.head_pitch = kContainerDownHeadPitch;
    target.sensor_settle_delay_sec = 1.6;
    target.configure_container_drop_z = true;
    target.container_drop_z_override_m = kContainerDownDropZOverride;
    target.container_roi_z_max_m = kContainerDownRoiZMax;
    target.bounds = {0.15, 1.35, 0.45, 0.08, kContainerDownSafeZMax, 1.50};
    if (container_top) {
      target.torso_height = kContainerTopTorsoHeight;
      target.head_pitch = kContainerTopHeadPitch;
      target.container_drop_z_override_m = kContainerTopDropZOverride;
      target.container_roi_z_max_m = kContainerTopRoiZMax;
      target.bounds.z_max = kContainerTopSafeZMax;
    }
  }

  if (explicit_shelf || (looks_like_shelf && !looks_like_container)) {
    target.mode = "shelf";
    target.frame = "shelf_drop_start";
    target.service_label = "/vision/shelf_drop_tf/enable";
    target.torso_height = 0.3;
    target.head_pitch = -0.45;
    target.sensor_settle_delay_sec = 0.8;
    target.bounds = {0.35, 0.95, 0.60, 0.30, 1.80, 1.10};
    if (contains("shelf_3")) {
      target.shelf_level_rank = 1;
      target.torso_height = 0.3;
      target.head_pitch = 0.0;
    } else if (contains("shelf_2")) {
      target.shelf_level_rank = 2;
      target.torso_height = 0.2;
      target.head_pitch = -0.35;
    } else if (contains("shelf_1")) {
      target.shelf_level_rank = 3;
      target.torso_height = 0.1;
      target.head_pitch = -0.65;
    } else if (contains("top") || contains("upper") || contains("alto") ||
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
    target.head_pitch = -0.7;
    target.bounds = {0.15, 0.80, 0.55, 0.35, 1.40, 1.05};
  }

  return target;
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

std::string FindSpace::owlFrameName(const std::string &class_name, int instance) {
  std::string sanitized = class_name;
  std::replace(sanitized.begin(), sanitized.end(), ' ', '_');
  return "OWL_" + sanitized + "_" + std::to_string(instance);
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
