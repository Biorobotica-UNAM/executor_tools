#include "executor_tools/Drop.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

#include "tf2/exceptions.h"

using namespace std::chrono_literals;

Drop::Drop(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Manipulation> &manipulation,
           const std::shared_ptr<Hardware> &hardware,
           const std::shared_ptr<Knowledge> &knowledge,
           const std::shared_ptr<HRI> &hri)
    : node_(node), manipulation_(manipulation), hardware_(hardware),
      knowledge_(knowledge), hri_(hri) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

Result Drop::run(const std::string &argument,
                 const std::shared_ptr<GoalHandle> &gh) {
  if (!gh || !gh->is_active()) {
    return {false, "FAILED", "goal_handle_inactive"};
  }

  const DropTarget drop_target = resolveDropTarget(argument);

  std::ostringstream ready_detail;
  ready_detail << "arg=" << (argument.empty() ? "<empty>" : argument)
               << " mode=" << drop_target.mode
               << " frame=" << drop_target.frame;
  publishFeedback(gh, DropState::READY, ready_detail.str());

  publishFeedback(gh, DropState::VERIFYING_HAND);
  const json left_object = knowledge_->get_world_state("robot", "", "left_hand");
  const json right_object =
      knowledge_->get_world_state("robot", "", "right_hand");

  if (!left_object.is_string() || !right_object.is_string()) {
    publishFeedback(gh, DropState::FAILED, "invalid_hand_state");
    return {false, "FAILED", "invalid_hand_state"};
  }

  std::string hand_error;
  auto held = selectHeldObject(argument, left_object.get<std::string>(),
                               right_object.get<std::string>(), hand_error);
  if (!held) {
    publishFeedback(gh, DropState::FAILED, hand_error);
    return {false, "FAILED", hand_error};
  }

  arm_group_ = held->arm_group;
  const std::string object_name = resolveObjectName(held->value);
  const std::string tip = selectTip(object_name);

  RCLCPP_INFO(node_->get_logger(),
              "[Drop] soltando object='%s' hand=%s arm=%s tip=%s target=%s",
              held->value.c_str(), held->hand.c_str(), arm_group_.c_str(),
              tip.c_str(), drop_target.frame.c_str());

  hardware_->MoveBase(0.0, 0.0, 0.0);

  std::string pose_error;
  auto safe_pose = waitForSafeDropPose(gh, drop_target, pose_error);
  if (!safe_pose) {
    RCLCPP_WARN(node_->get_logger(),
                "[Drop] no se pudo obtener pose segura desde TF/knowledge: %s",
                pose_error.c_str());
    publishFeedback(gh, DropState::FAILED, pose_error);
    return {false, "FAILED", pose_error};
  }

  publishFeedback(gh, DropState::VALIDATE_DROP_TF,
                  "source=" + safe_pose->source);

  if (!poseInBounds(drop_target, safe_pose->pose)) {
    const auto &p = safe_pose->pose.pose.position;
    const double distance_xy = std::hypot(p.x, p.y);
    std::ostringstream oss;
    oss << "drop_pose_out_of_bounds x=" << p.x << " y=" << p.y
        << " z=" << p.z << " dist_xy=" << distance_xy
        << " mode=" << drop_target.mode << " source=" << safe_pose->source;
    publishFeedback(gh, DropState::FAILED, oss.str());
    return {false, "FAILED", oss.str()};
  }

  auto release_pose = safe_pose->pose;
  release_pose.pose.position.z += RELEASE_Z_OFFSET;
  release_pose.header.stamp = node_->now();

  RCLCPP_INFO(node_->get_logger(),
              "[Drop] pose liberacion source=%s frame=%s "
              "position=(%.3f %.3f %.3f) quat=(%.3f %.3f %.3f %.3f)",
              safe_pose->source.c_str(), release_pose.header.frame_id.c_str(),
              release_pose.pose.position.x, release_pose.pose.position.y,
              release_pose.pose.position.z, release_pose.pose.orientation.x,
              release_pose.pose.orientation.y, release_pose.pose.orientation.z,
              release_pose.pose.orientation.w);

  publishFeedback(gh, DropState::PREPARING_RELEASE);
  if (!hri_->Say("Preparing drop")) {
    publishFeedback(gh, DropState::PREPARING_RELEASE, "say_failed");
  }

  const bool torso_ok = hardware_->MoveTorso(0.3);
  const bool prepare_arm_ok =
      (arm_group_ == "left_arm_group")
          ? hardware_->MoveLeftArm({PREPARE_POSE[0], PREPARE_POSE[1],
                                    PREPARE_POSE[2], PREPARE_POSE[3],
                                    PREPARE_POSE[4], PREPARE_POSE[5]})
          : hardware_->MoveRightArm({PREPARE_POSE[0], PREPARE_POSE[1],
                                     PREPARE_POSE[2], PREPARE_POSE[3],
                                     PREPARE_POSE[4], PREPARE_POSE[5]});

  if (!torso_ok || !prepare_arm_ok) {
    publishFeedback(gh, DropState::FAILED, "invalid_preparation_pose");
    return {false, "FAILED", "invalid_preparation_pose"};
  }

  if (!gh->is_active()) {
    publishFeedback(gh, DropState::FAILED, "goal_cancelled");
    return {false, "FAILED", "goal_cancelled"};
  }

  publishFeedback(gh, DropState::PLANNING_RELEASE);
  if (!hri_->Say("Planning drop trajectory")) {
    publishFeedback(gh, DropState::PLANNING_RELEASE, "say_failed");
  }

  std::string msg;
  bool ok = manipulation_->PlanTrajectory(release_pose, arm_group_, tip,
                                          joint_names_, waypoints_, msg);
  if (!ok) {
    publishFeedback(gh, DropState::FAILED, msg);
    return {false, "FAILED", msg};
  }

  if (!gh->is_active()) {
    publishFeedback(gh, DropState::FAILED, "goal_cancelled");
    return {false, "FAILED", "goal_cancelled"};
  }

  publishFeedback(gh, DropState::EXECUTING_RELEASE);
  ok = manipulation_->FollowTrajectory(arm_group_, joint_names_, waypoints_, msg);
  if (!ok) {
    publishFeedback(gh, DropState::FAILED, msg);
    return {false, "FAILED", msg};
  }

  if (!gh->is_active()) {
    publishFeedback(gh, DropState::FAILED, "goal_cancelled");
    return {false, "FAILED", "goal_cancelled"};
  }

  publishFeedback(gh, DropState::OPENING_GRIPPER);
  const bool gripper_ok =
      (held->hand == "left_hand") ? hardware_->MoveLeftGripper("open")
                                  : hardware_->MoveRightGripper("open");
  if (!gripper_ok) {
    publishFeedback(gh, DropState::FAILED, "gripper_open_failed");
    return {false, "FAILED", "gripper_open_failed"};
  }

  publishFeedback(gh, DropState::REPORTING_OCCUPANCY);
  const bool report_ok =
      knowledge_->update_world_state("robot", "", held->hand, "nothing");
  if (!report_ok) {
    publishFeedback(gh, DropState::FAILED, "report_object_failed");
    return {false, "FAILED", "report_object_failed"};
  }

  publishFeedback(gh, DropState::PLANNING_RETRACT);
  const auto home = retractPose(arm_group_);
  if (!hri_->Say("Planning retract")) {
    publishFeedback(gh, DropState::PLANNING_RETRACT, "say_failed");
  }

  ok = manipulation_->PlanTrajectory(home, arm_group_, tip, joint_names_,
                                     waypoints_, msg);
  if (!ok) {
    publishFeedback(gh, DropState::FAILED, msg);
    return {false, "FAILED", msg};
  }

  if (!gh->is_active()) {
    publishFeedback(gh, DropState::FAILED, "goal_cancelled");
    return {false, "FAILED", "goal_cancelled"};
  }

  publishFeedback(gh, DropState::EXECUTING_RETRACT);
  ok = manipulation_->FollowTrajectory(arm_group_, joint_names_, waypoints_, msg);
  if (!ok) {
    publishFeedback(gh, DropState::FAILED, msg);
    return {false, "FAILED", msg};
  }

  publishFeedback(gh, DropState::PREPARING_TRANSPORT);
  const bool transport_torso_ok = hardware_->MoveTorso(0.3);
  const bool transport_arm_ok =
      (arm_group_ == "left_arm_group")
          ? hardware_->MoveLeftArm({TRANSPORT_POSE[0], TRANSPORT_POSE[1],
                                    TRANSPORT_POSE[2], TRANSPORT_POSE[3],
                                    TRANSPORT_POSE[4], TRANSPORT_POSE[5]})
          : hardware_->MoveRightArm({TRANSPORT_POSE[0], TRANSPORT_POSE[1],
                                     TRANSPORT_POSE[2], TRANSPORT_POSE[3],
                                     TRANSPORT_POSE[4], TRANSPORT_POSE[5]});

  if (!transport_torso_ok || !transport_arm_ok) {
    publishFeedback(gh, DropState::FAILED, "transport_pose_failed");
    return {false, "FAILED", "transport_pose_failed"};
  }

  if (!gh->is_active()) {
    publishFeedback(gh, DropState::FAILED, "goal_cancelled");
    return {false, "FAILED", "goal_cancelled"};
  }

  publishFeedback(gh, DropState::SUCCESS, arm_group_);
  return {true, "SUCCESS", arm_group_ + "_drop_completed"};
}

std::optional<Drop::HeldObject> Drop::selectHeldObject(
    const std::string &argument, const std::string &left_object,
    const std::string &right_object, std::string &error) const {
  const std::string arg = normalizeToken(argument);
  const std::string left = normalizeToken(left_object);
  const std::string right = normalizeToken(right_object);
  const bool left_occupied = left != "nothing" && !left.empty();
  const bool right_occupied = right != "nothing" && !right.empty();

  const auto make_left = [&]() {
    return HeldObject{"left_hand", "left_arm_group", left_object};
  };
  const auto make_right = [&]() {
    return HeldObject{"right_hand", "right_arm_group", right_object};
  };

  const bool requests_left = arg.rfind("left_", 0) == 0 ||
                             arg.find("_left_") != std::string::npos;
  const bool requests_right = arg.rfind("right_", 0) == 0 ||
                              arg.find("_right_") != std::string::npos;

  if (requests_left && !requests_right) {
    if (!left_occupied) {
      error = "selected_hand_empty:left_hand";
      return std::nullopt;
    }
    return make_left();
  }

  if (requests_right && !requests_left) {
    if (!right_occupied) {
      error = "selected_hand_empty:right_hand";
      return std::nullopt;
    }
    return make_right();
  }

  const auto mentions = [&arg](const std::string &held) {
    if (held.empty() || held == "nothing") {
      return false;
    }
    if (arg.find(held) != std::string::npos) {
      return true;
    }
    const auto sep = held.find('_');
    if (sep != std::string::npos && sep + 1 < held.size()) {
      return arg.find(held.substr(sep + 1)) != std::string::npos;
    }
    return false;
  };

  if (mentions(left) && left_occupied) {
    return make_left();
  }
  if (mentions(right) && right_occupied) {
    return make_right();
  }

  if (left_occupied && !right_occupied) {
    return make_left();
  }
  if (right_occupied && !left_occupied) {
    return make_right();
  }

  if (!left_occupied && !right_occupied) {
    error = "no_object_to_drop";
  } else {
    error = "ambiguous_hand";
  }
  return std::nullopt;
}

std::optional<Drop::SafeDropPose> Drop::waitForSafeDropPose(
    const std::shared_ptr<GoalHandle> &gh, const DropTarget &drop_target,
    std::string &error) const {
  const std::string target_frame = "base_link";
  publishFeedback(gh, DropState::WAIT_DROP_TF,
                  target_frame + " <- " + drop_target.frame);

  geometry_msgs::msg::TransformStamped tf;
  std::string last_error;
  int attempts = 0;
  const auto start_time = std::chrono::steady_clock::now();
  while (gh->is_active() && std::chrono::steady_clock::now() - start_time < 3s) {
    ++attempts;
    try {
      tf = tf_buffer_->lookupTransform(target_frame, drop_target.frame,
                                       tf2::TimePointZero, 500ms);
      SafeDropPose safe_pose;
      safe_pose.pose = transformToPose(tf, node_->now());
      safe_pose.source = "tf:" + drop_target.frame;
      return safe_pose;
    } catch (const tf2::TransformException &ex) {
      last_error = ex.what();
      std::this_thread::sleep_for(100ms);
    }
  }

  if (!gh->is_active()) {
    error = "goal_cancelled";
    return std::nullopt;
  }

  RCLCPP_WARN(node_->get_logger(),
              "[Drop] TF %s no disponible despues de %d intento(s): %s",
              drop_target.frame.c_str(), attempts, last_error.c_str());

  std::string knowledge_error;
  auto safe_pose = readSafeDropPoseFromKnowledge(drop_target, knowledge_error);
  if (safe_pose) {
    return safe_pose;
  }

  error = "drop_tf_not_available_run_find_space_first";
  if (!knowledge_error.empty()) {
    error += ": " + knowledge_error;
  }
  return std::nullopt;
}

std::optional<Drop::SafeDropPose> Drop::readSafeDropPoseFromKnowledge(
    const DropTarget &drop_target, std::string &error) const {
  const json mode = knowledge_->get_world_state("robot", "", "safe_drop_mode");
  const json frame = knowledge_->get_world_state("robot", "", "safe_drop_frame");
  const json pose = knowledge_->get_world_state("robot", "", "safe_drop_pose");

  if (!mode.is_string() || !frame.is_string() || !pose.is_array() ||
      pose.size() != 7) {
    error = "safe_drop_knowledge_empty";
    return std::nullopt;
  }

  if (frame.get<std::string>() != drop_target.frame ||
      mode.get<std::string>() != drop_target.mode) {
    error = "safe_drop_knowledge_mismatch mode=" + mode.get<std::string>() +
            " frame=" + frame.get<std::string>();
    return std::nullopt;
  }

  SafeDropPose safe_pose;
  safe_pose.pose.header.stamp = node_->now();
  safe_pose.pose.header.frame_id = "base_link";
  safe_pose.pose.pose.position.x = pose[0].get<double>();
  safe_pose.pose.pose.position.y = pose[1].get<double>();
  safe_pose.pose.pose.position.z = pose[2].get<double>();
  safe_pose.pose.pose.orientation.x = pose[3].get<double>();
  safe_pose.pose.pose.orientation.y = pose[4].get<double>();
  safe_pose.pose.pose.orientation.z = pose[5].get<double>();
  safe_pose.pose.pose.orientation.w = pose[6].get<double>();
  safe_pose.source = "knowledge:" + frame.get<std::string>();
  return safe_pose;
}

std::string Drop::resolveObjectName(const std::string &held_object) const {
  std::string category;
  std::string index;
  if (parseObjectId(held_object, category, index)) {
    const json object_name =
        knowledge_->get_world_state(category, index, "name");
    if (object_name.is_string() && !object_name.get<std::string>().empty()) {
      return object_name.get<std::string>();
    }
  }

  std::string fallback = normalizeToken(held_object);
  if (fallback.rfind("left_", 0) == 0) {
    fallback = fallback.substr(5);
  } else if (fallback.rfind("right_", 0) == 0) {
    fallback = fallback.substr(6);
  }

  const auto last_sep = fallback.rfind('_');
  if (last_sep != std::string::npos && last_sep + 1 < fallback.size() &&
      std::all_of(fallback.begin() + last_sep + 1, fallback.end(),
                  [](unsigned char c) { return std::isdigit(c); })) {
    fallback = fallback.substr(0, last_sep);
  }

  return fallback.empty() ? held_object : fallback;
}

std::string Drop::selectTip(const std::string &object_name) const {
  const bool is_pinch = PINCH_OBJECTS.count(normalizeToken(object_name)) > 0;
  if (arm_group_ == "left_arm_group") {
    return is_pinch ? "left_pinch_object_link" : "left_grab_object_link";
  }
  return is_pinch ? "right_pinch_object_link" : "right_grab_object_link";
}

Drop::PoseStamped Drop::retractPose(const std::string &arm) const {
  PoseStamped pose;
  pose.header.stamp = node_->now();
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.4;
  pose.pose.position.y = (arm == "right_arm_group") ? -0.25 : 0.25;
  pose.pose.position.z = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = -0.707;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.707;
  return pose;
}

Drop::DropTarget Drop::resolveDropTarget(const std::string &argument) {
  std::string lower = argument;
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });

  const auto contains = [&lower](const std::string &needle) {
    return lower.find(needle) != std::string::npos;
  };

  DropTarget target;
  target.mode = "table";
  target.frame = "table_drop_start";
  target.bounds = {0.15, 1.20, 0.80, 0.35, 1.40, 1.40};

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
    target.bounds = {0.15, 1.35, 0.45, 0.08, 1.10, 1.50};
  }

  if (explicit_shelf || (looks_like_shelf && !looks_like_container)) {
    target.mode = "shelf";
    target.frame = "shelf_drop_start";
    target.bounds = {0.15, 1.35, 0.85, 0.30, 1.70, 1.55};
  }

  if (explicit_table) {
    target.mode = "table";
    target.frame = "table_drop_start";
    target.bounds = {0.15, 1.20, 0.80, 0.35, 1.40, 1.40};
  }

  return target;
}

void Drop::publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                           DropState state, const std::string &detail) const {
  if (!gh->is_active()) {
    return;
  }

  std::string msg = "[Drop] " + stateName(state);
  if (!detail.empty()) {
    msg += ": " + detail;
  }

  auto fb = std::make_shared<ExecuteTask::Feedback>();
  fb->feedback.message = msg;
  gh->publish_feedback(fb);

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

std::string Drop::stateName(DropState s) {
  switch (s) {
  case DropState::READY:
    return "READY";
  case DropState::VERIFYING_HAND:
    return "VERIFYING_HAND";
  case DropState::WAIT_DROP_TF:
    return "WAIT_DROP_TF";
  case DropState::VALIDATE_DROP_TF:
    return "VALIDATE_DROP_TF";
  case DropState::PREPARING_RELEASE:
    return "PREPARING_RELEASE";
  case DropState::PLANNING_RELEASE:
    return "PLANNING_RELEASE";
  case DropState::EXECUTING_RELEASE:
    return "EXECUTING_RELEASE";
  case DropState::OPENING_GRIPPER:
    return "OPENING_GRIPPER";
  case DropState::REPORTING_OCCUPANCY:
    return "REPORTING_OCCUPANCY";
  case DropState::PLANNING_RETRACT:
    return "PLANNING_RETRACT";
  case DropState::EXECUTING_RETRACT:
    return "EXECUTING_RETRACT";
  case DropState::PREPARING_TRANSPORT:
    return "PREPARING_TRANSPORT";
  case DropState::SUCCESS:
    return "SUCCESS";
  case DropState::FAILED:
    return "FAILED";
  default:
    return "UNKNOWN";
  }
}

std::string Drop::normalizeToken(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  std::replace(value.begin(), value.end(), ' ', '_');
  return value;
}

bool Drop::parseObjectId(const std::string &object_id, std::string &category,
                         std::string &index) {
  const std::size_t sep = object_id.rfind('_');
  if (sep == std::string::npos || sep == 0 || sep + 1 == object_id.size()) {
    return false;
  }
  category = object_id.substr(0, sep);
  index = object_id.substr(sep + 1);
  return true;
}

bool Drop::poseInBounds(const DropTarget &drop_target,
                        const PoseStamped &pose) {
  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double z = pose.pose.position.z;
  const double distance_xy = std::hypot(x, y);
  const auto &b = drop_target.bounds;
  return x >= b.x_min && x <= b.x_max && std::fabs(y) <= b.abs_y_max &&
         z >= b.z_min && z <= b.z_max && distance_xy <= b.dist_xy_max;
}

Drop::PoseStamped Drop::transformToPose(
    const geometry_msgs::msg::TransformStamped &tf, const rclcpp::Time &stamp) {
  PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = tf.header.frame_id.empty() ? "base_link"
                                                    : tf.header.frame_id;
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = tf.transform.translation.z;
  pose.pose.orientation.x = tf.transform.rotation.x;
  pose.pose.orientation.y = tf.transform.rotation.y;
  pose.pose.orientation.z = tf.transform.rotation.z;
  pose.pose.orientation.w = tf.transform.rotation.w;
  return pose;
}
