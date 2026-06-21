#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"
#include "hri_tools/HRI.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "manipulation_tools/Manipulation.hpp"
#include "nlohmann/json.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

enum class DropState {
  READY,
  VERIFYING_HAND,
  WAIT_DROP_TF,
  VALIDATE_DROP_TF,
  PREPARING_RELEASE,
  PLANNING_RELEASE,
  EXECUTING_RELEASE,
  MOVING_FORWARD,
  OPENING_GRIPPER,
  REPORTING_OCCUPANCY,
  MOVING_BACKWARD,
  PLANNING_RETRACT,
  EXECUTING_RETRACT,
  PREPARING_TRANSPORT,
  FLOOR_EXTENDING_ARM,
  FLOOR_LOWERING_TORSO,
  FLOOR_OPENING_GRIPPER,
  FLOOR_RAISING_TORSO,
  FLOOR_CLOSING_GRIPPER,
  FLOOR_RESETTING_ARM,
  SUCCESS,
  FAILED
};

// Drop consume una TF ya generada por FindSpace y ejecuta la maniobra inversa
// de Take: mover la mano ocupada al espacio seguro, abrir gripper y marcar la
// mano como libre en knowledge.
class Drop {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using json = nlohmann::json;

  Drop(const rclcpp::Node::SharedPtr &node,
       const std::shared_ptr<Manipulation> &manipulation,
       const std::shared_ptr<Hardware> &hardware,
       const std::shared_ptr<Knowledge> &knowledge,
       const std::shared_ptr<HRI> &hri);

  Result run(const std::string &argument,
             const std::shared_ptr<GoalHandle> &gh);

private:
  struct DropBounds {
    double x_min{0.15};
    double x_max{1.20};
    double abs_y_max{0.80};
    double z_min{0.35};
    double z_max{1.40};
    double dist_xy_max{1.40};
  };

  struct DropTarget {
    std::string mode;
    std::string frame;
    DropBounds bounds;
  };

  struct HeldObject {
    std::string hand;
    std::string arm_group;
    std::string value;
  };

  struct SafeDropPose {
    PoseStamped pose;
    std::string source;
  };

  struct MotorStage {
    std::string label;
    std::vector<std::size_t> motors;
    std::size_t waypoints;
  };

  struct ReleasePoseCandidate {
    std::string label;
    double dx;
    double dy;
    double dz;
  };

  static constexpr std::array<double, 6> PREPARE_POSE = {-0.9, 0.0, 0.0, 1.5, 0.0, 0.9};
  static constexpr std::array<double, 6> TRANSPORT_POSE = {-0.9, 0.0, 0.0, 1.5, 0.0, 0.9};

  static constexpr std::array<double, 6> FLOOR_ARM_EXTEND = {0.4, 0.1, 0.3, 1.0, 0.0, 0.2};
  static constexpr std::array<double, 6> FLOOR_ARM_RESET  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  static constexpr double MOVE_DISTANCE_M = 0.02;
  static constexpr double MOVE_LINEAR_SPEED = 0.05;

  inline static const std::unordered_set<std::string> PINCH_OBJECTS = {
      "spoon", "fork", "knife", "chopstick", "teaspoon",
      "dessert_spoon", "butter_knife", "spatula", "ladle",
      "pen", "pencil", "marker", "straw", "chopsticks",
      "card", "coin", "key", "remote",
  };

  void publishFeedback(const std::shared_ptr<GoalHandle> &gh, DropState state,
                       const std::string &detail = "") const;

  std::optional<HeldObject> selectHeldObject(
      const std::string &argument, const std::string &left_object,
      const std::string &right_object, std::string &error) const;
  std::optional<SafeDropPose> waitForSafeDropPose(
      const std::shared_ptr<GoalHandle> &gh, const DropTarget &drop_target,
      std::string &error) const;
  std::optional<SafeDropPose> readSafeDropPoseFromKnowledge(
      const DropTarget &drop_target, std::string &error) const;
  std::string resolveObjectName(const std::string &held_object) const;
  std::string selectTip(const std::string &object_name) const;
  PoseStamped retractPose(const std::string &arm) const;
  std::vector<MotorStage> releaseMotorStages() const;
  std::vector<ReleasePoseCandidate> releasePoseCandidates() const;
  bool orderReleaseJointSequence(const std::vector<std::string> &joint_names,
                                 std::vector<double> &waypoints) const;
  bool buildReverseTrajectory(const std::vector<std::string> &joint_names,
                              const std::vector<double> &source_waypoints,
                              std::vector<double> &reversed_waypoints) const;
  std::size_t maxRemainingReleaseBlocks() const;
  std::string motorJointName(std::size_t motor) const;
  bool moveBase(double distance_m, const std::shared_ptr<GoalHandle> &gh);

  static DropTarget resolveDropTarget(const std::string &argument);
  static std::string stateName(DropState s);
  static std::string normalizeToken(std::string value);
  static bool parseObjectId(const std::string &object_id,
                            std::string &category, std::string &index);
  static bool poseInBounds(const DropTarget &drop_target,
                           const PoseStamped &pose);
  static PoseStamped transformToPose(
      const geometry_msgs::msg::TransformStamped &tf,
      const rclcpp::Time &stamp);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Manipulation> manipulation_;
  std::shared_ptr<Hardware> hardware_;
  std::shared_ptr<Knowledge> knowledge_;
  std::shared_ptr<HRI> hri_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string arm_group_;
  std::vector<std::string> joint_names_;
  std::vector<double> waypoints_;
};
