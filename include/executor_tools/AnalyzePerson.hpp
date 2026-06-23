#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "hardware_tools/Hardware.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "vision_tools/Vision.hpp"

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

enum class AnalyzePersonState {
  READY,
  MOVING_HEAD,
  STABILIZING,
  PARSING_ARGS,
  CALLING_SERVICE,
  UPDATING_KNOWLEDGE,
  SUCCESS,
  FAILED
};

class AnalyzePerson {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  AnalyzePerson(const rclcpp::Node::SharedPtr &node,
                const std::shared_ptr<Hardware> &hardware,
                const std::shared_ptr<Vision> &vision,
                const std::shared_ptr<Knowledge> &knowledge);

  Result run(const std::string &argument,
             const std::shared_ptr<GoalHandle> &gh);

private:
  bool runColor(const std::string &color, const std::string &attribute,
                std::string &out_msg,
                std::vector<Vision::YoloDetection> &out_detections,
                bool &out_has_point,
                geometry_msgs::msg::PointStamped &out_point);
  bool runGesture(const std::string &gesture_type, const std::string &arm_side,
                  std::string &out_msg,
                  std::vector<Vision::YoloDetection> &out_detections,
                  bool &out_has_point,
                  geometry_msgs::msg::PointStamped &out_point);
  bool runCombined(const std::string &gesture_type, const std::string &color,
                   const std::string &attribute, const std::string &arm_side,
                   std::string &out_msg,
                   std::vector<Vision::YoloDetection> &out_detections,
                   bool &out_has_point,
                   geometry_msgs::msg::PointStamped &out_point);
  bool updateUserNavigation(const std::vector<Vision::YoloDetection> &detections,
                            bool has_point,
                            const geometry_msgs::msg::PointStamped &point,
                            std::string &out_msg);

  void publishFeedback(const std::shared_ptr<GoalHandle> &gh,
                       AnalyzePersonState state,
                       const std::string &detail = "");

  static std::string stateName(AnalyzePersonState state);
  static nlohmann::json parsePersonString(const std::string &argument);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Hardware> hardware_;
  std::shared_ptr<Vision> vision_;
  std::shared_ptr<Knowledge> knowledge_;
};
