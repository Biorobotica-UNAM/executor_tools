#include "executor_tools/RequestAuthorization.hpp"

RequestAuthorization::RequestAuthorization(
  const rclcpp::Node::SharedPtr& node)
: node_(node)
{
  client_ =
    node_->create_client<AuthorizeTask>(
      "/supervisor_executor/authorize_task");
}

bool RequestAuthorization::request(
  const std::string& action,
  const std::string& argument,
  bool local_result,
  const std::string& context_message)
{
  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "Supervisor service not available.");
    return false;
  }

  auto req = std::make_shared<AuthorizeTask::Request>();
  req->action = action;
  req->argument = argument;
  req->local_result = local_result;
  req->context_message = context_message;

  auto future = client_->async_send_request(req);

  if (rclcpp::spin_until_future_complete(node_, future)
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "Authorization request failed.");
    return false;
  }

  return future.get()->authorized;
}

