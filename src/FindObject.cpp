#include "executor_tools/FindObject.hpp"
#include "executor_tools/Result.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────

FindObject::FindObject(const rclcpp::Node::SharedPtr &node,
                       const std::shared_ptr<Hardware> &hardware,
                       const std::shared_ptr<Vision> &vision,
                       const std::shared_ptr<Knowledge> &knowledge)
    : node_(node), hardware_(hardware), vision_(vision), knowledge_(knowledge) {
}

// ─────────────────────────────────────────────────────────────
//  run
// ─────────────────────────────────────────────────────────────

Result FindObject::run(const std::string &argument,
                       const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active())
    return {false, "FAILED", "goal_inactive"};

  publishFeedback(gh, FindObjectState::READY, argument);

  std::string query_name;
  std::string object_index;

  if (!parseArgument(argument, query_name, object_index)) {
    publishFeedback(gh, FindObjectState::FAILED, "invalid_argument");
    return {false, "FAILED", "invalid_argument"};
  }

  // parseArgument() puede resolver dos cosas distintas:
  //  - formato clasico "categoria_indice" -> query_name ES la categoria,
  //    y por lo tanto tambien es el termino correcto para la busqueda
  //    visual (ej. "fruits_1" -> busca "fruits").
  //  - nombre suelto, ej. "apple" -> query_name termina siendo la
  //    categoria donde vive esa entrada en el world state (ej.
  //    "objects"), pero para la busqueda visual queremos el nombre
  //    especifico que se pidio, no la categoria generica.
  // Si reconstruir "categoria_indice" no reproduce el argumento
  // original, es porque vino del camino de nombre suelto.
  const std::string reconstructed_legacy_arg = query_name + "_" + object_index;
  const std::string vision_query =
      (reconstructed_legacy_arg == argument) ? query_name : argument;

  for (const auto &[pan, tilt] : scan_poses_) {
    if (!gh->is_active())
      return {false, "FAILED", "goal_cancelled"};

    publishFeedback(gh, FindObjectState::MOVING_HEAD);
    hardware_->MoveHead({pan, tilt});
    std::this_thread::sleep_for(1500ms);

    publishFeedback(gh, FindObjectState::STABILIZING);
    std::this_thread::sleep_for(500ms);

    publishFeedback(gh, FindObjectState::DETECTING);

    // ── EMPTY CHAIR ────────────────────────────────────

    if (query_name == "seat") {
      geometry_msgs::msg::PointStamped seat_point;

      if (detectEmptyChair(seat_point)) {
        publishFeedback(gh, FindObjectState::LOOKING_AT_TARGET);

        for (int i = 0; i < 5; ++i) {
          hardware_->LookAt(seat_point);
          std::this_thread::sleep_for(20ms);
        }

        std::this_thread::sleep_for(1500ms);

        publishFeedback(gh, FindObjectState::SUCCESS, "empty_chair");
        return {true, "SUCCESS", "empty_chair_found"};
      }
    }

    // ── OWL OBJECTS ────────────────────────────────────

    else {
      geometry_msgs::msg::PoseStamped object_pose, object_nav;
      std::string detected_name;

      if (vision_->GetOwlPose(vision_query, object_pose, object_nav,
                              detected_name)) {
        publishFeedback(gh, FindObjectState::LOOKING_AT_TARGET, detected_name);

        for (int i = 0; i < 6; ++i) {
          hardware_->LookAt(object_pose);
          std::this_thread::sleep_for(20ms);
        }

        std::this_thread::sleep_for(1500ms);

        geometry_msgs::msg::PoseStamped centered_object_pose,
            centered_object_nav;
        std::string centered_detected_name;

        publishFeedback(gh, FindObjectState::DETECTING,
                        detected_name + "_centered");

        if (!refreshCenteredObjectPose(vision_query, centered_object_pose,
                                       centered_object_nav,
                                       centered_detected_name, gh)) {
          publishFeedback(gh, FindObjectState::FAILED,
                          "centered_pose_not_found");
          continue;
        }

        publishFeedback(gh, FindObjectState::UPDATING, centered_detected_name);
        updateObjectState(query_name, object_index, centered_detected_name,
                          centered_object_pose, centered_object_nav);

        publishFeedback(gh, FindObjectState::SUCCESS, centered_detected_name);
        return {true, "SUCCESS", centered_detected_name + "_found"};
      }
    }
  }

  publishFeedback(gh, FindObjectState::FAILED, vision_query);
  return {false, "FAILED", vision_query + "_not_found"};
}

// ─────────────────────────────────────────────────────────────
//  detectEmptyChair
// ─────────────────────────────────────────────────────────────

bool FindObject::detectEmptyChair(geometry_msgs::msg::PointStamped &point) {
  auto result = vision_->FindSeat();

  if (!result.first)
    return false;

  point = result.second;
  return true;
}

// ─────────────────────────────────────────────────────────────
//  refreshCenteredObjectPose
// ─────────────────────────────────────────────────────────────

bool FindObject::refreshCenteredObjectPose(
    const std::string &query_name, geometry_msgs::msg::PoseStamped &out_pose,
    geometry_msgs::msg::PoseStamped &out_nav, std::string &out_detected_name,
    const std::shared_ptr<GoalHandle> &gh) {
  for (int attempt = 1; attempt <= 3; ++attempt) {
    if (!gh || !gh->is_active())
      return false;

    if (vision_->GetOwlPose(query_name, out_pose, out_nav, out_detected_name,
                            2s)) {
      RCLCPP_INFO(
          node_->get_logger(),
          "[FindObject] Refreshed centered object pose for '%s' as '%s' "
          "(attempt %d)",
          query_name.c_str(), out_detected_name.c_str(), attempt);
      return true;
    }

    RCLCPP_WARN(node_->get_logger(),
                "[FindObject] Could not refresh centered object pose for '%s' "
                "(attempt %d)",
                query_name.c_str(), attempt);
    std::this_thread::sleep_for(200ms);
  }

  return false;
}

// ─────────────────────────────────────────────────────────────
//  parseArgument
// ─────────────────────────────────────────────────────────────

bool FindObject::parseArgument(const std::string &argument,
                               std::string &out_name,
                               std::string &out_index) const {
  // ── 1) Formato clasico "categoria_indice" (ej. "fruits_1", "seat_1")
  // Solo cuenta como indice si el sufijo tras el ultimo '_' es numerico;
  // si no lo es (p.ej. "juice_pack"), no es un indice valido y el
  // argumento se trata como nombre suelto en (2).
  const std::size_t sep = argument.rfind('_');

  if (sep != std::string::npos && sep != 0 && sep + 1 != argument.size()) {
    const std::string candidate_index = argument.substr(sep + 1);
    const bool index_is_numeric =
        std::all_of(candidate_index.begin(), candidate_index.end(),
                    [](unsigned char c) { return std::isdigit(c); });

    if (index_is_numeric) {
      out_name = argument.substr(0, sep);
      out_index = candidate_index;
      return true;
    }
  }

  // ── 2) Nombre de objeto suelto (ej. "apple", "knife") ──────────────
  // Se recorren las categorias de objetos del world state hasta
  // encontrar una entrada cuyo campo "name" coincida exactamente.
  // "objects" va primero porque es la categoria que CLIPS usa para
  // planear.
  static const std::vector<std::string> kNameSearchCategories = {"objects"};

  for (const auto &category : kNameSearchCategories) {
    for (int i = 1; i <= 200; ++i) {
      const std::string index_str = std::to_string(i);
      const nlohmann::json name_json =
          knowledge_->get_world_state(category, index_str, "name");

      if (name_json.empty())
        break; // numeracion contigua: no hay mas entradas aqui

      const std::string entry_name =
          name_json.is_string() ? name_json.get<std::string>() : "";

      if (entry_name == argument) {
        out_name = category;
        out_index = index_str;
        return true;
      }
    }
  }

  return false;
}

// ─────────────────────────────────────────────────────────────
//  updateObjectState
// ─────────────────────────────────────────────────────────────

void FindObject::updateObjectState(const std::string &category,
                                   const std::string &index,
                                   const std::string &name,
                                   const geometry_msgs::msg::PoseStamped &pose,
                                   const geometry_msgs::msg::PoseStamped &nav) {
  const auto &p = pose.pose.position;
  const auto &q = pose.pose.orientation;

  nlohmann::json manipulation = {p.x, p.y, p.z, q.x, q.y, q.z, q.w};

  const auto &np = nav.pose.position;
  const auto &nq = nav.pose.orientation;
  nlohmann::json navigation = {np.x, np.y, np.z, nq.x, nq.y, nq.z, nq.w};

  knowledge_->update_world_state(category, index, "name", name);
  knowledge_->update_world_state(category, index, "manipulation",
                                 manipulation.dump());
  knowledge_->update_world_state(category, index, "navigation",
                                 navigation.dump());
}

// ─────────────────────────────────────────────────────────────
//  publishFeedback
// ─────────────────────────────────────────────────────────────

void FindObject::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                                 FindObjectState state,
                                 const std::string &detail) {
  if (!gh->is_active())
    return;

  std::string msg = "[find_object] " + stateName(state);
  if (!detail.empty())
    msg += ": " + detail;

  auto fb = std::make_shared<ExecuteTask::Feedback>();
  fb->feedback.message = msg;
  gh->publish_feedback(fb);

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// ─────────────────────────────────────────────────────────────
//  stateName
// ─────────────────────────────────────────────────────────────

std::string FindObject::stateName(FindObjectState s) {
  switch (s) {
  case FindObjectState::READY:
    return "READY";
  case FindObjectState::MOVING_HEAD:
    return "MOVING_HEAD";
  case FindObjectState::STABILIZING:
    return "STABILIZING";
  case FindObjectState::DETECTING:
    return "DETECTING";
  case FindObjectState::LOOKING_AT_TARGET:
    return "LOOKING_AT_TARGET";
  case FindObjectState::UPDATING:
    return "UPDATING";
  case FindObjectState::SUCCESS:
    return "SUCCESS";
  case FindObjectState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}
