#include "executor_tools/FindPerson.hpp"

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
    person_index = std::to_string(getNextGuestIndex());
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

  // ── Scan positions ───────────────────────────────────────

  struct ScanPoint {
    double pan;
    double tilt;
  };
  std::vector<ScanPoint> scan_points;

  if (anybody) {
    for (double tilt : {0.0, -0.2})
      scan_points.push_back({0.0, tilt});
  } else {
    for (double pan : {0.0, -0.6, 0.6, 0.0})
      scan_points.push_back({pan, 0.0});
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
  hardware_->MoveHead({pan, tilt});

  for (int i = 0; i < 12; ++i) {
    std::this_thread::sleep_for(100ms);
    if (!gh->is_active())
      return false;
  }

  // ── 2. Detect + pose ──────────────────────────────────────
  publishFeedback(gh, FindPersonState::DETECTING);

  Vision::PersonDetection detection;

  if (!vision_->DetectPersonPose(detection))
    return false;

  if (!gh->is_active())
    return false;

  geometry_msgs::msg::PoseStamped target_pose = detection.pose;

  if (!gh->is_active())
    return false;

  // ── 3. Look at the person ─────────────────────────────────
  publishFeedback(gh, FindPersonState::LOOK_AT_TARGET);

  geometry_msgs::msg::PointStamped look_point;
  look_point.header = target_pose.header;
  look_point.point = target_pose.pose.position;
  hardware_->LookAt(look_point);

  for (int i = 0; i < 15; ++i) {
    std::this_thread::sleep_for(100ms);
    if (!gh->is_active())
      return false;
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

    found_name = identified_name;
    RCLCPP_INFO(node_->get_logger(), "[FindPerson] Person identified: %s",
                found_name.c_str());

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

  if (overwrite_name)
    knowledge_->update_world_state(category, index, "name", name);

  knowledge_->update_world_state(category, index, "description",
                                 desc_json.dump());
  knowledge_->update_world_state(category, index, "navigation", nav.dump());
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
