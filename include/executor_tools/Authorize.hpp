#pragma once

#include "rclcpp/rclcpp.hpp"
#include "supervisor_executor_interfaces/srv/authorize.hpp"

class RequestAuthorization
{
public:
  using Authorize =
      supervisor_executor_interfaces::srv::Authorize;

  explicit RequestAuthorization(
    const rclcpp::Node::SharedPtr& node);

  bool request(
    const std::string& action,
    const std::string& argument,
    bool local_result,
    const std::string& context_message);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Authorize>::SharedPtr client_;
};
