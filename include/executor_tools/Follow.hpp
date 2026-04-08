#pragma once

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"

class Follow {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  Follow(const rclcpp::Node::SharedPtr &node);

  bool run(const std::string &arg,
           const std::shared_ptr<GoalHandle> &goalHandle);

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client; // Cliente del servicio
  Executor executor;
};