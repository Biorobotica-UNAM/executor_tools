#pragma once
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "vision_tools/Vision.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>
#include <vector>

// ============================================================
// State machine for the FindPerson action
// ============================================================
enum class FindPersonState {
  READY,
  SCANNING,
  DETECTING,
  CALCULATING_POSE,
  LOOK_AT_TARGET,
  IDENTIFYING,
  REGISTERING,
  DESCRIBING,
  UPDATING,
  SUCCESS,
  FAILED
};

class FindPerson {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  FindPerson(const rclcpp::Node::SharedPtr &node,
             const std::shared_ptr<Hardware> &hardware,
             const std::shared_ptr<Vision> &vision,
             const std::shared_ptr<Knowledge> &knowledge);

  Result run(const std::string &argument,
             const std::shared_ptr<GoalHandle> &gh);

private:
  // Espera activa hasta que la cabeza deja de moverse.
  // Usa Hardware::GetCurrentPose("head", 0/1) para leer pan y tilt reales.
  // timeout_ms:     máximo tiempo de espera (ms)
  // tolerance_rad:  delta máximo entre muestras para considerar estable
  // stable_needed:  número de muestras consecutivas dentro del umbral
  bool waitForHeadStable(const std::shared_ptr<GoalHandle> &gh,
                         int    timeout_ms    = 2000,
                         double tolerance_rad = 0.04,
                         int    stable_needed = 2);

  bool scanPosition(double pan, double tilt, bool anybody,
                    const std::string &target, const std::string &category,
                    const std::string &person_index, std::string &found_name,
                    const std::shared_ptr<GoalHandle> &gh);

  void updatePersonState(const std::string &category, const std::string &index,
                         const std::string &name,
                         const geometry_msgs::msg::PoseStamped &pose,
                         const geometry_msgs::msg::PoseStamped &nav,
                         const std::string &description, bool overwrite_name);

  void publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                       FindPersonState state, const std::string &detail = "");

  static std::string stateName(FindPersonState s);

  uint64_t getNextGuestIndex();
  static std::string generatePersonName(uint64_t index);

  rclcpp::Node::SharedPtr   node_;
  std::shared_ptr<Hardware> hardware_;
  std::shared_ptr<Vision>   vision_;
  std::shared_ptr<Knowledge> knowledge_;
};