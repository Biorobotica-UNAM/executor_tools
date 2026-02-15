#include "executor_tools/Executor.hpp"

Executor::Executor(const rclcpp::Node::SharedPtr& node)
: node(node)
{
  client = node->create_client<AuthorizeTask>(
    "supervisor/supervisor_executor/authorize");
}

bool Executor::authorize(const std::string& action,
                         const std::string& argument,
                         const std::string& worker_result,
                         const std::string& context_message)
{
  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Supervisor authorization service not available.");
    return false;
  }

  auto req = std::make_shared<AuthorizeTask::Request>();
  req->action = action;
  req->argument = argument;
  req->worker_result = worker_result;
  req->context_message = context_message;

  auto future = client->async_send_request(req);

  if (rclcpp::spin_until_future_complete(node, future)
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Authorization request failed.");
    return false;
  }

  return future.get()->approved;
}

