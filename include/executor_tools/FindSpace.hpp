#pragma once

#include <chrono>
#include <optional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hardware_tools/Hardware.hpp"
#include "knowledge_tools/Knowledge.hpp"

#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vision_interfaces/srv/get_detections.hpp"

enum class FindSpaceState {
  READY,
  PREPARE_SENSOR,
  PRECENTER_CONTAINER_WITH_OWL,
  WAIT_SPACE_TF,
  VALIDATE_SPACE_TF,
  SUCCESS,
  FAILED
};

// FindSpace se encarga solo de encontrar y validar una TF de espacio seguro.
// No debe soltar objetos ni mover brazos: su producto final es una marca TF
// confiable como container_drop_start, table_drop_start o shelf_drop_start.
class FindSpace {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  FindSpace(const rclcpp::Node::SharedPtr &node,
       const std::shared_ptr<Hardware> &hardware,
       const std::shared_ptr<Knowledge> &knowledge);

  Result run(const std::string &argument,
             const std::shared_ptr<GoalHandle> &gh);

private:
  struct SpaceBounds {
    double x_min{0.15};
    double x_max{1.20};
    double abs_y_max{0.80};
    double z_min{0.35};
    double z_max{1.40};
    double dist_xy_max{1.40};
  };

  struct OwlContainerPose {
    // Datos de depuracion de OWLv2: bbox_center_x ayuda a saber si el objeto
    // estaba fuera del centro de imagen; x/y/z indican si la base debe ajustar
    // su pose antes de intentar el filtrado geometrico de nube.
    bool detected{false};
    std::string class_name;
    std::string frame;
    double confidence{0.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double bbox_center_x{0.0};
    double bbox_width{0.0};
    double bbox_height{0.0};
  };

  struct SpaceTarget {
    std::string mode;
    std::string frame;
    std::string service_label;
    double torso_height{0.3};
    double head_pitch{-0.6};
    double sensor_settle_delay_sec{0.3};
    int shelf_level_rank{0};
    SpaceBounds bounds;
  };

  void publishFeedback(const std::shared_ptr<GoalHandle> &gh, FindSpaceState state,
                       const std::string &detail = "");

  bool setSpaceTfNodeEnabled(
      const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr &client,
      const std::string &service_label, bool enabled);
  // Precentrado para contenedores: primero intenta corregir con cabeza para
  // mejorar la vista; solo usa pulsos cortos de base si la pose fisica sigue
  // fuera de la zona ideal pero aun dentro de la zona corregible.
  bool precenterContainerWithOwl(const std::shared_ptr<GoalHandle> &gh,
                                 const SpaceTarget &space_target);
  bool preScanShelf(const std::shared_ptr<GoalHandle> &gh,
                    const SpaceTarget &space_target);
  bool refineShelfAlignment(
      const std::shared_ptr<GoalHandle> &gh,
      const SpaceTarget &space_target,
      const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr &enable_client,
      geometry_msgs::msg::TransformStamped &tf);
  bool waitForFreshShelfTf(
      const std::shared_ptr<GoalHandle> &gh,
      const SpaceTarget &space_target,
      const rclcpp::Time &request_stamp,
      std::chrono::milliseconds max_wait,
      geometry_msgs::msg::TransformStamped &tf,
      std::string &last_error);
  bool requestOwlContainerPose(OwlContainerPose &pose_out,
                               std::string &error_out);
  void publishBasePulse(double vx, double vy, double wz,
                        std::chrono::milliseconds duration);
  bool storeSafeDropInKnowledge(
      const SpaceTarget &space_target,
      const geometry_msgs::msg::TransformStamped &tf);
  void configureTablePreferenceForHeldObject();
  void configureShelfPreference(const SpaceTarget &space_target);
  
  //category section
  std::optional<std::string> heldObjectName() const;
  std::optional<std::string> objectNameFromId(const std::string &object_id) const;
  std::unordered_map<std::string, std::vector<std::string>>
  loadObjectCategories() const;
  std::optional<std::string> categoryForObject(
      const std::string &object_name,
      const std::unordered_map<std::string, std::vector<std::string>>
          &categories) const;
  std::optional<double> preferredShelfYFromCategory(
      const std::string &held_category,
      const std::vector<std::string> &category_objects);

  static SpaceTarget resolveDropTarget(const std::string &argument);
  static std::string owlFrameName(const std::string &class_name, int instance);
  static std::string normalizeObjectToken(const std::string &name);
  static std::string stateName(FindSpaceState s);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Hardware> hardware_;
  std::shared_ptr<Knowledge> knowledge_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      enable_table_drop_tf_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      enable_container_drop_tf_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      enable_shelf_drop_tf_client_;
  rclcpp::Client<vision_interfaces::srv::GetDetections>::SharedPtr
      owl_v2_client_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
