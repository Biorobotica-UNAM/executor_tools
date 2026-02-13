#pragma once

#include "rclcpp/rclcpp.hpp"
#include "executor_interfaces/srv/authorize_task.hpp"

class RequestAuthorization
{
public:
  using AuthorizeTask =
      executor_interfaces::srv::AuthorizeTask;

  explicit RequestAuthorization(
    const rclcpp::Node::SharedPtr& node);

  bool request(
    const std::string& action,
    const std::string& argument,
    bool local_result,
    const std::string& context_message);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<AuthorizeTask>::SharedPtr client_;
};
