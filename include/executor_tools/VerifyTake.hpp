#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "vision_tools/Vision.hpp"

class VerifyTake
{
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  VerifyTake(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<Vision> & vision,
    const std::shared_ptr<Knowledge> & knowledge);

  Result run(
    const std::string & argument,
    const std::shared_ptr<GoalHandle> & gh);

private:
  bool parseArgument(
    const std::string & argument,
    std::string & category,
    std::string & index) const;

  void clearHeldObjectIfMatches(const std::string & object_id) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Vision> vision_;
  std::shared_ptr<Knowledge> knowledge_;
  double still_present_distance_m_{0.18};
};
