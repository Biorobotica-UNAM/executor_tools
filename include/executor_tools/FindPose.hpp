#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "hardware_tools/Hardware.hpp"
#include "hri_tools/HRI.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

enum class FindPoseState {
  READY,
  MOVING_HEAD,
  STABILIZING,
  DETECTING,
  LOOKING_AT_TARGET,
  UPDATING,
  SUCCESS,
  FAILED
};

class FindPose {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  FindPose(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Hardware> &hardware,
           const std::shared_ptr<HRI> &hri,
           const std::shared_ptr<Knowledge> &knowledge);

  Result run(const std::string &argument,
             const std::shared_ptr<GoalHandle> &gh);

private:
  void updateHumanPose(const std::string &action_type,
                       const std::string &direction,
                       const geometry_msgs::msg::PointStamped &point);

  void publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                       FindPoseState state, const std::string &detail = "");

  std::string stateName(FindPoseState s);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Hardware> hardware_;
  std::shared_ptr<HRI> hri_;
  std::shared_ptr<Knowledge> knowledge_;

  std::vector<std::pair<double, double>> scan_poses_ = {
      {0.0, -0.2}, {0.4, -0.2}, {-0.4, -0.2}};
};
