#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "supervisor_executor_interfaces/srv/authorize_task.hpp"

class Executor
{
public:
  using AuthorizeTask = supervisor_executor_interfaces::srv::AuthorizeTask;

  explicit Executor(const rclcpp::Node::SharedPtr& node);

  bool authorize(const std::string& action,
                 const std::string& argument,
                 const std::string& worker_result,
                 const std::string& context_message);

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<AuthorizeTask>::SharedPtr client;

  // --- sync wait (no spin) ---
  std::mutex mtx_;
  std::condition_variable cv_;
  bool waiting_{false};
  std::optional<bool> last_approved_;
  std::string last_message_;
};
