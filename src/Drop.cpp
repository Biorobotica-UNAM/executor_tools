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

std::vector<Drop::MotorStage> Drop::releaseMotorStages() const {
  return {
      {"shoulder_elbow", {0, 3}, 3},
      {"wrist", {5}, 2},
  };
}

std::vector<Drop::ReleasePoseCandidate> Drop::releasePoseCandidates() const {
  return {
      {"safe_space_nominal_front", 0.03, 0.00, 0.03},
      {"front_slightly_higher", 0.03, 0.00, 0.05},
      {"front_high_clearance", 0.03, 0.00, 0.07},
      {"farther_high_clearance", 0.03, 0.00, 0.09},
  };
}

bool Drop::moveBase(double distance_m,
                    const std::shared_ptr<GoalHandle> &gh) {
  const double sign = (distance_m >= 0.0) ? 1.0 : -1.0;
  const double duration_s = std::abs(distance_m) / MOVE_LINEAR_SPEED;

  rclcpp::Rate rate(30.0);
  const auto start = node_->now();

  while (rclcpp::ok() && (node_->now() - start).seconds() < duration_s) {
    if (!gh->is_active()) {
      hardware_->MoveBase(0.0, 0.0, 0.0);
      return false;
    }
    hardware_->MoveBase(sign * MOVE_LINEAR_SPEED, 0.0, 0.0);
    rate.sleep();
  }

  hardware_->MoveBase(0.0, 0.0, 0.0);
  return true;
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

  auto release_center_pose = safe_pose->pose;
  release_center_pose.header.stamp = node_->now();
  auto target_pose = release_center_pose;

  RCLCPP_INFO(node_->get_logger(),
              "[Drop] espacio seguro source=%s frame=%s "
              "position=(%.3f %.3f %.3f) quat=(%.3f %.3f %.3f %.3f)",
              safe_pose->source.c_str(), release_center_pose.header.frame_id.c_str(),
              release_center_pose.pose.position.x,
              release_center_pose.pose.position.y,
              release_center_pose.pose.position.z,
              release_center_pose.pose.orientation.x,
              release_center_pose.pose.orientation.y,
              release_center_pose.pose.orientation.z,
              release_center_pose.pose.orientation.w);

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
  bool ok = false;
  const auto pose_candidates = releasePoseCandidates();
  for (std::size_t i = 0; i < pose_candidates.size(); ++i) {
    const auto &candidate = pose_candidates[i];
    auto candidate_pose = release_center_pose;
    candidate_pose.pose.position.x += candidate.dx;
    candidate_pose.pose.position.y += candidate.dy;
    candidate_pose.pose.position.z += candidate.dz;

    RCLCPP_INFO(node_->get_logger(),
                "Trying drop release candidate %zu/%zu '%s': "
                "offset=[%.3f %.3f %.3f] target=[%.3f %.3f %.3f] "
                "orientation=[%.3f %.3f %.3f %.3f]",
                i + 1, pose_candidates.size(), candidate.label.c_str(),
                candidate.dx, candidate.dy, candidate.dz,
                candidate_pose.pose.position.x, candidate_pose.pose.position.y,
                candidate_pose.pose.position.z,
                candidate_pose.pose.orientation.x,
                candidate_pose.pose.orientation.y,
                candidate_pose.pose.orientation.z,
                candidate_pose.pose.orientation.w);

    ok = manipulation_->PlanTrajectory(
        candidate_pose, arm_group_, tip, joint_names_, waypoints_, msg, true,
        release_center_pose, 0.14, 0.24);

    if (ok) {
      target_pose = candidate_pose;
      RCLCPP_INFO(node_->get_logger(),
                  "Drop release candidate accepted: '%s'",
                  candidate.label.c_str());
      break;
    }

    RCLCPP_WARN(node_->get_logger(),
                "Drop release candidate failed: '%s' planner_message='%s'",
                candidate.label.c_str(), msg.c_str());

    if (!gh->is_active()) {
      publishFeedback(gh, DropState::FAILED, "goal_cancelled");
      return {false, "FAILED", "goal_cancelled"};
    }
  }

  if (!ok) {
    publishFeedback(gh, DropState::FAILED, msg);
    return {false, "FAILED", msg};
  }

  orderReleaseJointSequence(joint_names_, waypoints_);
  const auto release_joint_names = joint_names_;
  const auto release_waypoints = waypoints_;

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

  publishFeedback(gh, DropState::MOVING_FORWARD);
  if (!moveBase(MOVE_DISTANCE_M, gh)) {
    return {false, "FAILED", "goal_cancelled_during_moving_forward"};
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

  publishFeedback(gh, DropState::MOVING_BACKWARD);
  if (!moveBase(-MOVE_DISTANCE_M, gh)) {
    return {false, "FAILED", "goal_cancelled_during_moving_backward"};
  }

  publishFeedback(gh, DropState::PLANNING_RETRACT);
  const auto home = retractPose(arm_group_);
  if (!hri_->Say("Planning retract")) {
    publishFeedback(gh, DropState::PLANNING_RETRACT, "say_failed");
  }

  std::vector<double> reverse_waypoints;
  ok = buildReverseTrajectory(release_joint_names, release_waypoints,
                              reverse_waypoints);

  if (ok) {
    joint_names_ = release_joint_names;
    waypoints_ = reverse_waypoints;
    msg = "Using reversed release trajectory for retract";
    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "Could not build reversed release trajectory. Falling back to "
                "MoveIt retract planning with released-object exclusion.");

    RCLCPP_INFO(node_->get_logger(),
                "Planning drop retract fallback with released-object exclusion "
                "center=[%.3f %.3f %.3f] radius_xy=%.3f height_z=%.3f",
                target_pose.pose.position.x, target_pose.pose.position.y,
                target_pose.pose.position.z, 0.26, 0.40);

    ok = manipulation_->PlanTrajectory(home, arm_group_, tip, joint_names_,
                                       waypoints_, msg, true, target_pose, 0.26,
                                       0.40);

    if (!ok) {
      RCLCPP_WARN(node_->get_logger(),
                  "Drop retract fallback planning failed with primary "
                  "exclusion (%s). Retrying with wider released-object "
                  "exclusion radius_xy=%.3f height_z=%.3f",
                  msg.c_str(), 0.34, 0.50);

      ok = manipulation_->PlanTrajectory(home, arm_group_, tip, joint_names_,
                                         waypoints_, msg, true, target_pose,
                                         0.34, 0.50);
    }
  }

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

bool Drop::orderReleaseJointSequence(
    const std::vector<std::string> &joint_names,
    std::vector<double> &waypoints) const {
  if (joint_names.empty() || waypoints.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "Cannot order release joints: empty trajectory.");
    return false;
  }

  const std::size_t block_size = joint_names.size();
  if (waypoints.size() % block_size != 0) {
    RCLCPP_WARN(node_->get_logger(),
                "Cannot order release joints: %zu waypoint values are not a "
                "multiple of %zu joints.",
                waypoints.size(), block_size);
    return false;
  }

  const std::size_t blocks = waypoints.size() / block_size;
  if (blocks < 2) {
    RCLCPP_WARN(node_->get_logger(),
                "Cannot order release joints: only %zu block(s) available.",
                blocks);
    return false;
  }

  struct ResolvedStage {
    std::string label;
    std::vector<std::size_t> joint_indices;
    std::vector<std::string> joint_names;
    std::size_t waypoints{1};
  };

  const auto motor_stages = releaseMotorStages();
  std::vector<ResolvedStage> resolved_stages;
  resolved_stages.reserve(motor_stages.size());

  for (const auto &stage : motor_stages) {
    ResolvedStage resolved;
    resolved.label = stage.label;
    resolved.waypoints = stage.waypoints;

    if (resolved.waypoints == 0) {
      RCLCPP_WARN(node_->get_logger(),
                  "Cannot order release joints: stage '%s' has zero waypoints.",
                  stage.label.c_str());
      return false;
    }

    for (const auto motor : stage.motors) {
      const std::string joint_name = motorJointName(motor);
      if (joint_name.empty()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Cannot order release joints: motor %zu has no joint "
                    "mapping.",
                    motor);
        return false;
      }

      const auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
      if (it == joint_names.end()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Cannot order release joints: joint '%s' for motor %zu is "
                    "not present in trajectory.",
                    joint_name.c_str(), motor);
        return false;
      }

      resolved.joint_indices.push_back(
          static_cast<std::size_t>(std::distance(joint_names.begin(), it)));
      resolved.joint_names.push_back(joint_name);
    }

    if (resolved.joint_indices.empty()) {
      RCLCPP_WARN(node_->get_logger(),
                  "Cannot order release joints: stage '%s' has no motors.",
                  stage.label.c_str());
      return false;
    }

    resolved_stages.push_back(resolved);
  }

  const auto initial_first = waypoints.begin();
  const auto initial_last = initial_first + block_size;
  const auto final_first = waypoints.begin() + (blocks - 1) * block_size;

  std::vector<double> initial_block(initial_first, initial_last);
  std::vector<double> final_block(final_first, final_first + block_size);
  const std::size_t max_remaining_blocks = maxRemainingReleaseBlocks();

  std::vector<double> ordered_waypoints;
  std::size_t configured_stage_waypoints = 0;
  for (const auto &stage : resolved_stages) {
    configured_stage_waypoints += stage.waypoints;
  }
  ordered_waypoints.reserve(
      (configured_stage_waypoints + max_remaining_blocks) * block_size);

  std::vector<double> staged_block = initial_block;
  for (std::size_t i = 0; i < resolved_stages.size(); ++i) {
    const auto &stage = resolved_stages[i];
    const auto stage_start_block = staged_block;

    for (std::size_t step = 1; step <= stage.waypoints; ++step) {
      const double ratio =
          static_cast<double>(step) / static_cast<double>(stage.waypoints);
      for (const auto joint_index : stage.joint_indices) {
        const double start = stage_start_block[joint_index];
        const double goal = final_block[joint_index];
        staged_block[joint_index] = start + ratio * (goal - start);
      }
      ordered_waypoints.insert(ordered_waypoints.end(), staged_block.begin(),
                               staged_block.end());
    }

    for (std::size_t j = 0; j < stage.joint_indices.size(); ++j) {
      const std::size_t joint_index = stage.joint_indices[j];
      RCLCPP_INFO(node_->get_logger(),
                  "Release joint order stage %zu/%zu: %s joint=%s "
                  "waypoints=%zu initial=%.3f final=%.3f delta=%.3f",
                  i + 1, resolved_stages.size(), stage.label.c_str(),
                  stage.joint_names[j].c_str(), stage.waypoints,
                  stage_start_block[joint_index], final_block[joint_index],
                  final_block[joint_index] - stage_start_block[joint_index]);
    }
  }

  const std::size_t remaining_blocks = blocks - 1;
  const std::size_t kept_remaining_blocks =
      std::min(remaining_blocks, max_remaining_blocks);

  for (std::size_t kept = 1; kept <= kept_remaining_blocks; ++kept) {
    std::size_t block = 1;
    if (kept_remaining_blocks > 1) {
      block =
          1 + ((kept - 1) * (remaining_blocks - 1)) / (kept_remaining_blocks - 1);
    }

    std::vector<double> ordered_block(
        waypoints.begin() + block * block_size,
        waypoints.begin() + (block + 1) * block_size);
    for (const auto &stage : resolved_stages) {
      for (const std::size_t joint_index : stage.joint_indices) {
        ordered_block[joint_index] = final_block[joint_index];
      }
    }
    ordered_waypoints.insert(ordered_waypoints.end(), ordered_block.begin(),
                             ordered_block.end());
  }

  const std::size_t ordered_blocks = ordered_waypoints.size() / block_size;
  waypoints = std::move(ordered_waypoints);
  RCLCPP_INFO(node_->get_logger(),
              "Release trajectory reordered: original_blocks=%zu "
              "ordered_blocks=%zu configured_stages=%zu stage_waypoints=%zu "
              "kept_remaining=%zu/%zu. Retract reverse will use the opposite "
              "order.",
              blocks, ordered_blocks, resolved_stages.size(),
              configured_stage_waypoints, kept_remaining_blocks,
              remaining_blocks);

  return true;
}

std::size_t Drop::maxRemainingReleaseBlocks() const {
  return 4;
}

std::string Drop::motorJointName(std::size_t motor) const {
  const bool left = arm_group_ == "left_arm_group";

  switch (motor) {
  case 0:
    return left ? "lf_shoulder_joint" : "rf_shoulder_joint";
  case 1:
    return left ? "ls_shoulder_joint" : "rs_shoulder_joint";
  case 2:
    return left ? "l_arm_twist_joint" : "r_arm_twist_joint";
  case 3:
    return left ? "l_elbow_joint" : "r_elbow_joint";
  case 4:
    return left ? "l_hand_joint" : "r_hand_joint";
  case 5:
    return left ? "l_wrist_joint" : "r_wrist_joint";
  default:
    return "";
  }
}

bool Drop::buildReverseTrajectory(
    const std::vector<std::string> &joint_names,
    const std::vector<double> &source_waypoints,
    std::vector<double> &reversed_waypoints) const {
  reversed_waypoints.clear();

  if (joint_names.empty() || source_waypoints.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "Cannot build reverse trajectory: empty joint list or "
                "waypoints.");
    return false;
  }

  const std::size_t block_size = joint_names.size();
  if (source_waypoints.size() % block_size != 0) {
    RCLCPP_WARN(node_->get_logger(),
                "Cannot build reverse trajectory: %zu waypoint values are not "
                "a multiple of %zu joints.",
                source_waypoints.size(), block_size);
    return false;
  }

  const std::size_t blocks = source_waypoints.size() / block_size;
  if (blocks < 2) {
    RCLCPP_WARN(node_->get_logger(),
                "Cannot build reverse trajectory: only %zu block(s) available.",
                blocks);
    return false;
  }

  reversed_waypoints.reserve(source_waypoints.size());
  for (std::size_t reverse_block = blocks; reverse_block > 0; --reverse_block) {
    const std::size_t source_block = reverse_block - 1;
    const auto first = source_waypoints.begin() + source_block * block_size;
    const auto last = first + block_size;
    reversed_waypoints.insert(reversed_waypoints.end(), first, last);
  }

  const std::string wrist_joint =
      (arm_group_ == "left_arm_group") ? "l_wrist_joint" : "r_wrist_joint";
  const auto wrist_it =
      std::find(joint_names.begin(), joint_names.end(), wrist_joint);

  if (wrist_it != joint_names.end()) {
    const std::size_t wrist_index =
        static_cast<std::size_t>(std::distance(joint_names.begin(), wrist_it));
    const double retract_start_wrist = reversed_waypoints[wrist_index];
    const double retract_end_wrist =
        reversed_waypoints[(blocks - 1) * block_size + wrist_index];

    RCLCPP_INFO(node_->get_logger(),
                "Built reversed release retract: blocks=%zu joints=%zu "
                "wrist=%s start=%.3f end=%.3f",
                blocks, block_size, wrist_joint.c_str(), retract_start_wrist,
                retract_end_wrist);
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "Built reversed release retract: blocks=%zu joints=%zu "
                "wrist_joint_not_found=%s",
                blocks, block_size, wrist_joint.c_str());
  }

  return true;
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
    target.bounds = {0.15, 0.80, 0.55, 0.35, 1.40, 1.05};
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
  case DropState::MOVING_FORWARD:
    return "MOVING_FORWARD";
  case DropState::OPENING_GRIPPER:
    return "OPENING_GRIPPER";
  case DropState::REPORTING_OCCUPANCY:
    return "REPORTING_OCCUPANCY";
  case DropState::MOVING_BACKWARD:
    return "MOVING_BACKWARD";
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
