#include "executor_tools/Take.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <functional>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
Take::Take(
    const rclcpp::Node::SharedPtr&        node,
    const std::shared_ptr<Manipulation>&  manipulation,
    const std::shared_ptr<Hardware>&      hardware,
    const std::shared_ptr<Knowledge>&     knowledge,
    const std::shared_ptr<HRI>&           hri)
: node_(node),
  manipulation_(manipulation),
  hardware_(hardware),
  knowledge_(knowledge),
  hri_(hri)
{}

// ─────────────────────────────────────────────────────────────
//  approachMotorStages
// ─────────────────────────────────────────────────────────────
std::vector<Take::MotorStage> Take::approachMotorStages() const
{
    // Para cambiar el orden del acercamiento, modifica solo esta lista.
    // Cada entrada es una etapa; los motores dentro de la misma etapa se
    // adelantan juntos. El ultimo dato indica cuantos waypoints interpolados
    // usara esa etapa para llegar al valor final que calculo MoveIt.
    //
    // Datos que ocupa cada etapa:
    // - label: aparece en logs para identificar la prueba.
    // - motors: indices fisicos/logicos, traducidos por motorJointName().
    // - waypoints: pasos intermedios; mas pasos = movimiento mas gradual.
    //
    // Ejemplos utiles:
    //   { "shoulder_elbow", {0, 3}, 3 }, { "wrist", {5}, 2 }
    //   { "shoulder", {0}, 2 }, { "elbow_wrist", {3, 5}, 3 }
    // return {
    //     {"motor_0_shoulder", {0}, 1},
    //     {"motor_3_elbow", {3}, 1},
    //     {"motor_5_wrist", {5}, 1},
    // };
    return {
		{"arm", {1, 2, 3, 4}, 8},
		{"wrist", {5, 6}, 5},
		{"torso", {0}, 5},
	};
}

// ─────────────────────────────────────────────────────────────
//  approachPoseCandidates
// ─────────────────────────────────────────────────────────────
std::vector<Take::ApproachPoseCandidate> Take::approachPoseCandidates() const
{
    // Lista ordenada de metas de acercamiento para Take.
    //
    // La pose base viene de vision y representa el centro/punto de agarre del
    // objeto en base_link. Cada candidato aplica un offset pequeno sobre esa
    // pose antes de pedirle un plan a MoveIt.
    //
    // Que se busca con cada dato:
    // - dx: aleja la mano hacia el frente del robot para no pedir una meta tan
    //   metida en el objeto. Si MoveIt no puede samplear IK, aumentar dx suele
    //   dar mas espacio al brazo.
    // - dy: ajuste lateral fino. Normalmente queda en 0.0 para no cambiar de
    //   lado el agarre; puede servir si un brazo queda muy cruzado.
    // - dz: sube un poco la meta. Ayuda cuando la mesa, el borde del objeto o
    //   una orientacion de top grasp dejan la mano demasiado pegada.
    //
    // Orden recomendado: de menos invasivo a mas separado/alto. La primera pose
    // que planea se usa para ejecutar, y las siguientes ya no se intentan.
    return {
        {"vision_nominal_front", 0.01, 0.00, 0.02},
        {"front_slightly_higher", 0.01, 0.00, 0.03},
        {"front_high_clearance", 0.02, 0.00, 0.04},
        {"farther_high_clearance", 0.02, 0.00, 0.05},
    };
}

// ─────────────────────────────────────────────────────────────
//  moveBase
// ─────────────────────────────────────────────────────────────
bool Take::moveBase(double distance_m,
                     const std::shared_ptr<GoalHandle>& gh)
{
    const double sign       = (distance_m >= 0.0) ? 1.0 : -1.0;
    const double duration_s = std::abs(distance_m) / MOVE_LINEAR_SPEED;

    rclcpp::Rate rate(30.0);
    const auto   start = node_->now();

    while (rclcpp::ok() &&
           (node_->now() - start).seconds() < duration_s)
    {
        if (!gh->is_active())
        {
            hardware_->MoveBase(0.0, 0.0, 0.0);
            return false;
        }
        hardware_->MoveBase(sign * MOVE_LINEAR_SPEED, 0.0, 0.0);
        rate.sleep();
    }

    hardware_->MoveBase(0.0, 0.0, 0.0);
    return true;
}

// ─────────────────────────────────────────────────────────────
//  run
// ─────────────────────────────────────────────────────────────
Result Take::run(
    const std::string&                 argument,
    const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_inactive"};

    // ── READY ─────────────────────────────────────────────────
    publishFeedback(gh, TakeState::READY, argument);

    // ── GETTING_TARGET ────────────────────────────────────────
    publishFeedback(gh, TakeState::GETTING_TARGET);

    std::string category_name;
    std::string object_index;
    std::string hand;

    if (!parseArgument(argument, category_name, object_index))
    {
        publishFeedback(gh, TakeState::FAILED, "invalid_argument");
        return {false, "FAILED", "invalid_argument"};
    }

    json target_data = knowledge_->get_world_state(
                            category_name, object_index, "manipulation");

    if (target_data.empty())
    {
        publishFeedback(gh, TakeState::FAILED, "invalid_manipulation_data");
        return {false, "FAILED", "invalid_manipulation_data"};
    }

    // Fetch object name for grasp-type selection
    json object_name_json = knowledge_->get_world_state(
                                category_name, object_index, "name");

    const std::string object_name = object_name_json.is_string()
                                        ? object_name_json.get<std::string>()
                                        : "";

    if (object_name.empty())
        RCLCPP_WARN(node_->get_logger(),
            "Object name not found for '%s_%s' — defaulting to grab tip.",
            category_name.c_str(), object_index.c_str());
    else
        RCLCPP_INFO(node_->get_logger(),
            "Object name: '%s'", object_name.c_str());

    RCLCPP_INFO(node_->get_logger(),
        "Pose JSON: %s", target_data.dump().c_str());

    // ── VERIFYING_VACANCY ─────────────────────────────────────
    publishFeedback(gh, TakeState::VERIFYING_VACANCY);

    json left_object  = knowledge_->get_world_state("robot", "", "left_hand");
    json right_object = knowledge_->get_world_state("robot", "", "right_hand");

    if (left_object.empty() || right_object.empty())
    {
        publishFeedback(gh, TakeState::FAILED, "invalid_vacancy_data");
        return {false, "FAILED", "invalid_vacancy_data"};
    }

    const std::string target_id = category_name + "_" + object_index;
    const std::string left_val  = left_object.get<std::string>();
    const std::string right_val = right_object.get<std::string>();

    if (left_val == target_id || right_val == target_id)
    {
        arm_group_ = (left_val == target_id) ? "left_arm_group" : "right_arm_group";

        publishFeedback(gh, TakeState::VERIFYING_VACANCY, "already_holding_object");

        // ── PLANNING_RETRACT (early path) ─────────────────────
        publishFeedback(gh, TakeState::PLANNING_RETRACT);

        const auto home = retractPose(arm_group_);

        if (!hri_->Say("Already holding, retracting"))
            publishFeedback(gh, TakeState::PLANNING_RETRACT, "say_failed");

        std::string msg;
        bool ok = manipulation_->PlanTrajectory(
                        home, arm_group_, selectTip(object_name),
                        joint_names_, waypoints_, msg);

        if (!ok)
            return {false, "FAILED", msg};

        // ── EXECUTING_RETRACT (early path) ────────────────────
        publishFeedback(gh, TakeState::EXECUTING_RETRACT);

        ok = manipulation_->FollowTrajectory(
                    arm_group_, joint_names_, waypoints_, msg);
        if (!ok)
            return {false, "FAILED", msg};

        // ── PREPARING_TRANSPORT (early path) ──────────────────
        publishFeedback(gh, TakeState::PREPARING_TRANSPORT);

        bool torso_ok = hardware_->MoveTorso(0.3);
        bool arm_ok   = (arm_group_ == "left_arm_group")
            ? hardware_->MoveLeftArm({
                TRANSPORT_POSE[0], TRANSPORT_POSE[1], TRANSPORT_POSE[2],
                TRANSPORT_POSE[3], TRANSPORT_POSE[4], TRANSPORT_POSE[5]})
            : hardware_->MoveRightArm({
                TRANSPORT_POSE[0], TRANSPORT_POSE[1], TRANSPORT_POSE[2],
                TRANSPORT_POSE[3], TRANSPORT_POSE[4], TRANSPORT_POSE[5]});

        if (!arm_ok || !torso_ok)
            return {false, "FAILED", "transport_pose_failed"};

        if (!gh->is_active())
            return {false, "FAILED", "goal_cancelled"};

        // ── SUCCESS (early path) ──────────────────────────────
        publishFeedback(gh, TakeState::SUCCESS, arm_group_);
        return {true, "SUCCESS", arm_group_ + "_already_holding"};
    }

    // ── PREPARING_EXTEND ──────────────────────────────────────
    publishFeedback(gh, TakeState::PREPARING_EXTEND);

    const double y_coord = target_data[1].get<double>();

    if (!selectArm(arm_group_, hand,
                   right_object.get<std::string>(),
                   left_object.get<std::string>(),
                   y_coord))
    {
        publishFeedback(gh, TakeState::FAILED, "arm_not_available");
        return {false, "FAILED", "arm_not_available"};
    }

    // selectTip is now valid — arm_group_ is set
    const std::string tip = selectTip(object_name);

    RCLCPP_INFO(node_->get_logger(),
        "Selected arm: %s | tip: %s | target Y = %.3f",
        arm_group_.c_str(), tip.c_str(), y_coord);

    if (!hri_->Say("Preparing take"))
        publishFeedback(gh, TakeState::PREPARING_EXTEND, "say_failed");

    bool prepare_torso_ok = hardware_->MoveTorso(0.3);
    bool prepare_arm_ok   = (arm_group_ == "left_arm_group")
        ? hardware_->MoveLeftArm({
            PREPARE_POSE[0], PREPARE_POSE[1], PREPARE_POSE[2],
            PREPARE_POSE[3], PREPARE_POSE[4], PREPARE_POSE[5]})
        : hardware_->MoveRightArm({
            PREPARE_POSE[0], PREPARE_POSE[1], PREPARE_POSE[2],
            PREPARE_POSE[3], PREPARE_POSE[4], PREPARE_POSE[5]});

    if (!prepare_arm_ok || !prepare_torso_ok)
    {
        publishFeedback(gh, TakeState::FAILED, "invalid_preparation_pose");
        return {false, "FAILED", "invalid_preparation_pose"};
    }

    // ── OPENNING_GRIPPER ──────────────────────────────────────
    publishFeedback(gh, TakeState::OPENNING_GRIPPER);

    bool gripper_ok = (arm_group_ == "left_arm_group")
        ? hardware_->MoveLeftGripper("open")
        : hardware_->MoveRightGripper("open");

    if (!gripper_ok)
    {
        publishFeedback(gh, TakeState::OPENNING_GRIPPER, "gripper_open_failed");
        return {false, "FAILED", "gripper_open_failed"};
    }

    // ── PLANNING_EXTEND ───────────────────────────────────────
    publishFeedback(gh, TakeState::PLANNING_EXTEND);

    std::string msg;
    auto pose_opt = jsonToPose(target_data, node_->get_clock());

    if (!pose_opt)
    {
        publishFeedback(gh, TakeState::FAILED, "invalid_manipulation_pose");
        return {false, "FAILED", "invalid_manipulation_pose"};
    }

    auto object_exclusion_pose = pose_opt.value();
    auto target_pose = object_exclusion_pose;

    RCLCPP_INFO(node_->get_logger(),
        "Detected object pose for take:\n"
        "  frame_id: %s\n"
        "  position:    [%.3f, %.3f, %.3f]\n"
        "  orientation: [%.3f, %.3f, %.3f, %.3f]",
        target_pose.header.frame_id.c_str(),
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z,
        target_pose.pose.orientation.x,
        target_pose.pose.orientation.y,
        target_pose.pose.orientation.z,
        target_pose.pose.orientation.w);

    if (!hri_->Say("Planning trajectory"))
        publishFeedback(gh, TakeState::PLANNING_EXTEND, "say_failed");

    // Durante el acercamiento, la vision puede entregar una pose correcta para
    // el objeto pero demasiado estricta para IK/MoveIt. Probamos una lista de
    // pregrasp candidates con offsets pequenos, manteniendo object_exclusion_pose
    // como el centro real del objeto para no borrar mesa u otros obstaculos.
    bool ok = false;
    const auto pose_candidates = approachPoseCandidates();
    for (std::size_t i = 0; i < pose_candidates.size(); ++i)
    {
        const auto& candidate = pose_candidates[i];
        auto candidate_pose = object_exclusion_pose;
        candidate_pose.pose.position.x += candidate.dx;
        candidate_pose.pose.position.y += candidate.dy;
        candidate_pose.pose.position.z += candidate.dz;

        RCLCPP_INFO(node_->get_logger(),
            "Trying take approach candidate %zu/%zu '%s': offset=[%.3f %.3f %.3f] target=[%.3f %.3f %.3f] orientation=[%.3f %.3f %.3f %.3f]",
            i + 1,
            pose_candidates.size(),
            candidate.label.c_str(),
            candidate.dx,
            candidate.dy,
            candidate.dz,
            candidate_pose.pose.position.x,
            candidate_pose.pose.position.y,
            candidate_pose.pose.position.z,
            candidate_pose.pose.orientation.x,
            candidate_pose.pose.orientation.y,
            candidate_pose.pose.orientation.z,
            candidate_pose.pose.orientation.w);

        ok = manipulation_->PlanTrajectory(
                candidate_pose, arm_group_, tip,
                joint_names_, waypoints_, msg,
                true, object_exclusion_pose, 0.14, 0.24);

        if (ok)
        {
            target_pose = candidate_pose;
            RCLCPP_INFO(node_->get_logger(),
                "Take approach candidate accepted: '%s'",
                candidate.label.c_str());
            break;
        }

        RCLCPP_WARN(node_->get_logger(),
            "Take approach candidate failed: '%s' planner_message='%s'",
            candidate.label.c_str(),
            msg.c_str());

        if (!gh->is_active())
            return {false, "FAILED", "goal_cancelled"};
    }

    if (!ok)
        return {false, "FAILED", msg};

    // Reordenamos el inicio del acercamiento: primero hombro, luego codo,
    // luego muneca, y despues el resto de la trayectoria planeada. Asi la mano
    // no rota hasta el final ni inicia sola antes de acomodar el brazo.
    orderApproachJointSequence(joint_names_, waypoints_);
    const auto approach_joint_names = joint_names_;
    const auto approach_waypoints = waypoints_;

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── EXECUTING_EXTEND ──────────────────────────────────────
    publishFeedback(gh, TakeState::EXECUTING_EXTEND);

    ok = manipulation_->FollowTrajectory(
                arm_group_, joint_names_, waypoints_, msg);

    if (!ok)
        return {false, "FAILED", msg};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── MOVING_FORWARD ───────────────────────────────────────
    publishFeedback(gh, TakeState::MOVING_FORWARD);

    if (!moveBase(MOVE_DISTANCE_M, gh))
        return {false, "FAILED", "goal_cancelled_during_moving_forward"};

    // ── CLOSING_GRIPPER ───────────────────────────────────────
    publishFeedback(gh, TakeState::CLOSING_GRIPPER);

    gripper_ok = (arm_group_ == "left_arm_group")
        ? hardware_->MoveLeftGripper("close")
        : hardware_->MoveRightGripper("close");

    if (!gripper_ok)
    {
        publishFeedback(gh, TakeState::FAILED, "gripper_close_failed");
        return {false, "FAILED", "gripper_close_failed"};
    }

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};
    

    // ── REPORTING_OCCUPANCY ───────────────────────────────────
    publishFeedback(gh, TakeState::REPORTING_OCCUPANCY);

    bool report_ok = knowledge_->update_world_state(
                        "robot", "",
                        hand, category_name + "_" + object_index);

    if (!report_ok)
    {
        publishFeedback(gh, TakeState::FAILED, "report_object_failed");
        return {false, "FAILED", "report_object_failed"};
    }

	// ── MOVING_BACKWARD ──────────────────────────────────────────
    publishFeedback(gh, TakeState::MOVING_BACKWARD);

    if (!moveBase(-MOVE_DISTANCE_M, gh))
        return {false, "FAILED", "goal_cancelled_during_moving_backward"};

    // ── PLANNING_RETRACT ──────────────────────────────────────
    publishFeedback(gh, TakeState::PLANNING_RETRACT);

    const auto home = retractPose(arm_group_);

    if (!hri_->Say("Planning retract"))
        publishFeedback(gh, TakeState::PLANNING_RETRACT, "say_failed");

    std::vector<double> reverse_waypoints;
    ok = buildReverseTrajectory(
            approach_joint_names,
            approach_waypoints,
            reverse_waypoints);

    if (ok)
    {
        joint_names_ = approach_joint_names;
        waypoints_ = reverse_waypoints;
        msg = "Using reversed approach trajectory for retract";
        RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(),
            "Could not build reversed approach trajectory. Falling back to MoveIt retract planning with carried-object exclusion.");

        // Respaldo: si por alguna razon no se puede invertir la trayectoria,
        // usamos el plan de MoveIt con una exclusion local alrededor del objeto
        // agarrado para evitar falsos contactos con el gripper.
        auto retract_exclusion_pose = target_pose;
        RCLCPP_INFO(node_->get_logger(),
            "Planning retract fallback with carried-object exclusion center=[%.3f %.3f %.3f] radius_xy=%.3f height_z=%.3f",
            retract_exclusion_pose.pose.position.x,
            retract_exclusion_pose.pose.position.y,
            retract_exclusion_pose.pose.position.z,
            0.26,
            0.40);

        ok = manipulation_->PlanTrajectory(
                    home, arm_group_, tip,
                    joint_names_, waypoints_, msg,
                    true, retract_exclusion_pose, 0.26, 0.40);

        if (!ok)
        {
            RCLCPP_WARN(node_->get_logger(),
                "Retract fallback planning failed with primary exclusion (%s). Retrying with wider carried-object exclusion radius_xy=%.3f height_z=%.3f",
                msg.c_str(),
                0.34,
                0.50);

            ok = manipulation_->PlanTrajectory(
                        home, arm_group_, tip,
                        joint_names_, waypoints_, msg,
                        true, retract_exclusion_pose, 0.34, 0.50);
        }
    }

    if (!ok)
        return {false, "FAILED", msg};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── EXECUTING_RETRACT ─────────────────────────────────────
    publishFeedback(gh, TakeState::EXECUTING_RETRACT);

    ok = manipulation_->FollowTrajectory(
                arm_group_, joint_names_, waypoints_, msg);

    if (!ok)
        return {false, "FAILED", msg};

    // ── PREPARING_TRANSPORT ───────────────────────────────────
    publishFeedback(gh, TakeState::PREPARING_TRANSPORT);

    bool torso_ok = hardware_->MoveTorso(0.3);
    bool arm_ok   = (arm_group_ == "left_arm_group")
        ? hardware_->MoveLeftArm({
            TRANSPORT_POSE[0], TRANSPORT_POSE[1], TRANSPORT_POSE[2],
            TRANSPORT_POSE[3], TRANSPORT_POSE[4], TRANSPORT_POSE[5]})
        : hardware_->MoveRightArm({
            TRANSPORT_POSE[0], TRANSPORT_POSE[1], TRANSPORT_POSE[2],
            TRANSPORT_POSE[3], TRANSPORT_POSE[4], TRANSPORT_POSE[5]});

    if (!torso_ok || !arm_ok)
        return {false, "FAILED", "transport_pose_failed"};

    if (!gh->is_active())
        return {false, "FAILED", "goal_cancelled"};

    // ── SUCCESS ───────────────────────────────────────────────
    publishFeedback(gh, TakeState::SUCCESS, arm_group_);
    return {true, "SUCCESS", arm_group_ + "_take_completed"};
}

// ─────────────────────────────────────────────────────────────
//  parseArgument
// ─────────────────────────────────────────────────────────────
bool Take::parseArgument(
    const std::string& argument,
    std::string& out_name,
    std::string& out_index) const
{
    const std::size_t sep = argument.rfind('_');

    if (sep == std::string::npos || sep == 0 || sep + 1 == argument.size())
        return false;

    out_name  = argument.substr(0, sep);
    out_index = argument.substr(sep + 1);
    return true;
}

// ─────────────────────────────────────────────────────────────
//  jsonToPose
// ─────────────────────────────────────────────────────────────
std::optional<geometry_msgs::msg::PoseStamped> Take::jsonToPose(
    const json& data,
    rclcpp::Clock::SharedPtr clock)
{
    if (!data.is_array() || data.size() != 7)
        return std::nullopt;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp    = clock->now();
    pose.header.frame_id = "base_link";

    pose.pose.position.x    = data[0].get<double>();
    pose.pose.position.y    = data[1].get<double>();
    pose.pose.position.z    = data[2].get<double>();
    pose.pose.orientation.x = data[3].get<double>();
    pose.pose.orientation.y = data[4].get<double>();
    pose.pose.orientation.z = data[5].get<double>();
    pose.pose.orientation.w = data[6].get<double>();

    return pose;
}

// ─────────────────────────────────────────────────────────────
//  selectArm
// ─────────────────────────────────────────────────────────────
bool Take::selectArm(
    std::string& arm_group,
    std::string& hand,
    const std::string& right_obj,
    const std::string& left_obj,
    const double y_coord) const
{
    arm_group = (y_coord >= 0.0) ? "left_arm_group" : "right_arm_group";

    if (arm_group == "left_arm_group") {
        hand = "left_hand";
        return (left_obj == "nothing");
    } else {
        hand = "right_hand";
        return (right_obj == "nothing");
    }
}

// ─────────────────────────────────────────────────────────────
//  selectTip
// ─────────────────────────────────────────────────────────────
std::string Take::selectTip(const std::string& object_name) const
{
    const bool is_pinch = PINCH_OBJECTS.count(object_name) > 0;

    if (arm_group_ == "left_arm_group")
        return is_pinch ? "left_pinch_object_link" : "left_grab_object_link";
    else
        return is_pinch ? "right_pinch_object_link" : "right_grab_object_link";
}

// ─────────────────────────────────────────────────────────────
//  retractPose
// ─────────────────────────────────────────────────────────────
geometry_msgs::msg::PoseStamped Take::retractPose(
    const std::string& arm) const
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id    = "base_link";
    pose.pose.position.x    =  0.4;
    pose.pose.position.y    = (arm == "right_arm_group") ? -0.25 : 0.25;
    pose.pose.position.z    =  1.0;
    pose.pose.orientation.x =  0.0;
    pose.pose.orientation.y = -0.707;
    pose.pose.orientation.z =  0.0;
    pose.pose.orientation.w =  0.707;
    return pose;
}

// ─────────────────────────────────────────────────────────────
//  orderApproachJointSequence
// ─────────────────────────────────────────────────────────────
bool Take::orderApproachJointSequence(
    const std::vector<std::string>& joint_names,
    std::vector<double>& waypoints) const
{
    if (joint_names.empty() || waypoints.empty())
    {
        RCLCPP_WARN(node_->get_logger(),
            "Cannot order approach joints: empty trajectory.");
        return false;
    }

    const std::size_t block_size = joint_names.size();
    if (waypoints.size() % block_size != 0)
    {
        RCLCPP_WARN(node_->get_logger(),
            "Cannot order approach joints: %zu waypoint values are not a multiple of %zu joints.",
            waypoints.size(), block_size);
        return false;
    }

    const std::size_t blocks = waypoints.size() / block_size;
    if (blocks < 2)
    {
        RCLCPP_WARN(node_->get_logger(),
            "Cannot order approach joints: only %zu block(s) available.",
            blocks);
        return false;
    }

    struct ResolvedStage
    {
        std::string label;
        std::vector<std::size_t> joint_indices;
        std::vector<std::string> joint_names;
        std::size_t waypoints{1};
    };

    const auto motor_stages = approachMotorStages();
    std::vector<ResolvedStage> resolved_stages;
    resolved_stages.reserve(motor_stages.size());

    for (const auto& stage : motor_stages)
    {
        ResolvedStage resolved;
        resolved.label = stage.label;
        resolved.waypoints = stage.waypoints;

        if (resolved.waypoints == 0)
        {
            RCLCPP_WARN(node_->get_logger(),
                "Cannot order approach joints: stage '%s' has zero waypoints.",
                stage.label.c_str());
            return false;
        }

        for (const auto motor : stage.motors)
        {
            const std::string joint_name = motorJointName(motor);
            if (joint_name.empty())
            {
                RCLCPP_WARN(node_->get_logger(),
                    "Cannot order approach joints: motor %zu has no joint mapping.",
                    motor);
                return false;
            }

            const auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
            if (it == joint_names.end())
            {
                RCLCPP_WARN(node_->get_logger(),
                    "Cannot order approach joints: joint '%s' for motor %zu is not present in trajectory.",
                    joint_name.c_str(),
                    motor);
                return false;
            }

            resolved.joint_indices.push_back(
                static_cast<std::size_t>(std::distance(joint_names.begin(), it)));
            resolved.joint_names.push_back(joint_name);
        }

        if (resolved.joint_indices.empty())
        {
            RCLCPP_WARN(node_->get_logger(),
                "Cannot order approach joints: stage '%s' has no motors.",
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
    const std::size_t max_remaining_blocks = maxRemainingApproachBlocks();

    std::vector<double> ordered_waypoints;
    std::size_t configured_stage_waypoints = 0;
    for (const auto& stage : resolved_stages)
        configured_stage_waypoints += stage.waypoints;
    ordered_waypoints.reserve(
        (configured_stage_waypoints + max_remaining_blocks) * block_size);

    std::vector<double> staged_block = initial_block;
    for (std::size_t i = 0; i < resolved_stages.size(); ++i)
    {
        const auto& stage = resolved_stages[i];
        const auto stage_start_block = staged_block;

        // Cada etapa interpola solo sus motores desde el valor actual hasta el
        // valor final del ultimo bloque planeado por MoveIt. Los demas motores
        // conservan el valor acumulado por etapas previas.
        for (std::size_t step = 1; step <= stage.waypoints; ++step)
        {
            const double ratio =
                static_cast<double>(step) / static_cast<double>(stage.waypoints);
            for (const auto joint_index : stage.joint_indices)
            {
                const double start = stage_start_block[joint_index];
                const double goal = final_block[joint_index];
                staged_block[joint_index] = start + ratio * (goal - start);
            }
            ordered_waypoints.insert(
                ordered_waypoints.end(), staged_block.begin(), staged_block.end());
        }

        for (std::size_t j = 0; j < stage.joint_indices.size(); ++j)
        {
            const std::size_t joint_index = stage.joint_indices[j];
            RCLCPP_INFO(node_->get_logger(),
                "Approach joint order stage %zu/%zu: %s joint=%s waypoints=%zu initial=%.3f final=%.3f delta=%.3f",
                i + 1,
                resolved_stages.size(),
                stage.label.c_str(),
                stage.joint_names[j].c_str(),
                stage.waypoints,
                stage_start_block[joint_index],
                final_block[joint_index],
                final_block[joint_index] - stage_start_block[joint_index]);
        }
    }

    const std::size_t remaining_blocks = blocks - 1;
    const std::size_t kept_remaining_blocks =
        std::min(remaining_blocks, max_remaining_blocks);

    for (std::size_t kept = 1; kept <= kept_remaining_blocks; ++kept)
    {
        std::size_t block = 1;
        if (kept_remaining_blocks > 1)
        {
            block = 1 + ((kept - 1) * (remaining_blocks - 1)) /
                          (kept_remaining_blocks - 1);
        }

        std::vector<double> ordered_block(
            waypoints.begin() + block * block_size,
            waypoints.begin() + (block + 1) * block_size);
        for (const auto& stage : resolved_stages)
            for (const std::size_t joint_index : stage.joint_indices)
                ordered_block[joint_index] = final_block[joint_index];
        ordered_waypoints.insert(
            ordered_waypoints.end(), ordered_block.begin(), ordered_block.end());
    }

    const std::size_t ordered_blocks = ordered_waypoints.size() / block_size;
    waypoints = std::move(ordered_waypoints);
    RCLCPP_INFO(node_->get_logger(),
        "Approach trajectory reordered: original_blocks=%zu ordered_blocks=%zu configured_stages=%zu stage_waypoints=%zu kept_remaining=%zu/%zu. Retract reverse will use the opposite order.",
        blocks,
        ordered_blocks,
        resolved_stages.size(),
        configured_stage_waypoints,
        kept_remaining_blocks,
        remaining_blocks);

    return true;
}

// ─────────────────────────────────────────────────────────────
//  maxRemainingApproachBlocks
// ─────────────────────────────────────────────────────────────
std::size_t Take::maxRemainingApproachBlocks() const
{
    // Cantidad maxima de bloques originales de MoveIt que se conservan DESPUES
    // de las etapas manuales. No controla los waypoints por motor; eso se hace
    // en approachMotorStages(). Subirlo conserva mas detalle de la trayectoria
    // planeada; bajarlo hace el acercamiento mas directo.
    return 3;
}

// ─────────────────────────────────────────────────────────────
//  motorJointName
// ─────────────────────────────────────────────────────────────
std::string Take::motorJointName(std::size_t motor) const
{
    const bool left = arm_group_ == "left_arm_group";

	switch (motor)
	{
		case 0: return "torso_joint";
		case 1: return left ? "lf_shoulder_joint" : "rf_shoulder_joint";
		case 2: return left ? "ls_shoulder_joint" : "rs_shoulder_joint";
		case 3: return left ? "l_arm_twist_joint" : "r_arm_twist_joint";
		case 4: return left ? "l_elbow_joint" : "r_elbow_joint";
		case 5: return left ? "l_hand_joint" : "r_hand_joint";
		case 6: return left ? "l_wrist_joint" : "r_wrist_joint";

		default: return "";
	}
}

// ─────────────────────────────────────────────────────────────
//  buildReverseTrajectory
// ─────────────────────────────────────────────────────────────
bool Take::buildReverseTrajectory(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& source_waypoints,
    std::vector<double>& reversed_waypoints) const
{
    reversed_waypoints.clear();

    if (joint_names.empty() || source_waypoints.empty())
    {
        RCLCPP_WARN(node_->get_logger(),
            "Cannot build reverse trajectory: empty joint list or waypoints.");
        return false;
    }

    const std::size_t block_size = joint_names.size();
    if (source_waypoints.size() % block_size != 0)
    {
        RCLCPP_WARN(node_->get_logger(),
            "Cannot build reverse trajectory: %zu waypoint values are not a multiple of %zu joints.",
            source_waypoints.size(), block_size);
        return false;
    }

    const std::size_t blocks = source_waypoints.size() / block_size;
    if (blocks < 2)
    {
        RCLCPP_WARN(node_->get_logger(),
            "Cannot build reverse trajectory: only %zu block(s) available.",
            blocks);
        return false;
    }

    reversed_waypoints.reserve(source_waypoints.size());
    for (std::size_t reverse_block = blocks; reverse_block > 0; --reverse_block)
    {
        const std::size_t source_block = reverse_block - 1;
        const auto first = source_waypoints.begin() + source_block * block_size;
        const auto last = first + block_size;
        reversed_waypoints.insert(reversed_waypoints.end(), first, last);
    }

    const std::string wrist_joint =
        (arm_group_ == "left_arm_group") ? "l_wrist_joint" : "r_wrist_joint";
    const auto wrist_it =
        std::find(joint_names.begin(), joint_names.end(), wrist_joint);

    if (wrist_it != joint_names.end())
    {
        const std::size_t wrist_index =
            static_cast<std::size_t>(std::distance(joint_names.begin(), wrist_it));
        const double retract_start_wrist = reversed_waypoints[wrist_index];
        const double retract_end_wrist =
            reversed_waypoints[(blocks - 1) * block_size + wrist_index];

        RCLCPP_INFO(node_->get_logger(),
            "Built reversed approach retract: blocks=%zu joints=%zu wrist=%s start=%.3f end=%.3f",
            blocks,
            block_size,
            wrist_joint.c_str(),
            retract_start_wrist,
            retract_end_wrist);
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(),
            "Built reversed approach retract: blocks=%zu joints=%zu wrist_joint_not_found=%s",
            blocks,
            block_size,
            wrist_joint.c_str());
    }

    return true;
}

// ─────────────────────────────────────────────────────────────
//  publishFeedback
// ─────────────────────────────────────────────────────────────
void Take::publishFeedback(
    const std::shared_ptr<GoalHandle>& gh,
    TakeState                          state,
    const std::string&                 detail)
{
    if (!gh->is_active())
        return;

    std::string msg = "[take] " + stateName(state);
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
std::string Take::stateName(TakeState s)
{
    switch (s)
    {
        case TakeState::READY:               return "READY";
        case TakeState::GETTING_TARGET:      return "GETTING_TARGET";
        case TakeState::VERIFYING_VACANCY:   return "VERIFYING_VACANCY";
        case TakeState::PREPARING_EXTEND:    return "PREPARING_EXTEND";
        case TakeState::OPENNING_GRIPPER:    return "OPENNING_GRIPPER";
        case TakeState::PLANNING_EXTEND:     return "PLANNING_EXTEND";
        case TakeState::EXECUTING_EXTEND:    return "EXECUTING_EXTEND";
		case TakeState::MOVING_FORWARD:     return "MOVING_FORWARD";
        case TakeState::CLOSING_GRIPPER:     return "CLOSING_GRIPPER";
        case TakeState::REPORTING_OCCUPANCY: return "REPORTING_OCCUPANCY";
		case TakeState::MOVING_BACKWARD:        return "MOVING_BACKWARD";
        case TakeState::PLANNING_RETRACT:    return "PLANNING_RETRACT";
        case TakeState::EXECUTING_RETRACT:   return "EXECUTING_RETRACT";
        case TakeState::PREPARING_TRANSPORT: return "PREPARING_TRANSPORT";
        case TakeState::SUCCESS:             return "SUCCESS";
        case TakeState::FAILED:              return "FAILED";
        default:                             return "UNKNOWN";
    }
}
