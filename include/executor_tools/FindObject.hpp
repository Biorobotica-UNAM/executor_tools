#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"

#include "hardware_tools/Hardware.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "vision_tools/Vision.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <memory>
#include <string>
#include <vector>

enum class FindObjectState {
  READY,
  MOVING_HEAD,
  STABILIZING,
  DETECTING,
  LOOKING_AT_TARGET,
  UPDATING,
  SUCCESS,
  FAILED
};

class FindObject {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  FindObject(const rclcpp::Node::SharedPtr &node,
             const std::shared_ptr<Hardware> &hardware,
             const std::shared_ptr<Vision> &vision,
             const std::shared_ptr<Knowledge> &knowledge);

  Result run(const std::string &argument,
             const std::shared_ptr<GoalHandle> &gh);

private:
  bool detectEmptyChair(geometry_msgs::msg::PointStamped &point);

  bool parseArgument(const std::string &argument, std::string &out_name,
                     std::string &out_index) const;

  bool refreshCenteredObjectPose(const std::string &query_name,
                                 geometry_msgs::msg::PoseStamped &out_pose,
                                 std::string &out_detected_name,
                                 const std::shared_ptr<GoalHandle> &gh);

  void updateObjectState(const std::string &category, const std::string &index,
                         const std::string &name,
                         const geometry_msgs::msg::PoseStamped &pose);

  void publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                       FindObjectState state, const std::string &detail = "");

  std::string stateName(FindObjectState s);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Hardware> hardware_;
  std::shared_ptr<Vision> vision_;
  std::shared_ptr<Knowledge> knowledge_;

  const std::vector<std::pair<double, double>> scan_poses_ = {
      {0.0, -0.60}, {-0.25, -0.65}, {0.25, -0.65}};
};
