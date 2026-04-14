#include "executor_tools/Align.hpp"

Align::Align(const rclcpp::Node::SharedPtr &node)
    : node(node), executor(node) {
  client = node->create_client<std_srvs::srv::Trigger>("/tasks/table_align");
}

bool Align::run(const std::string &argument,
               const std::shared_ptr<GoalHandle> &goalHandle) {

  // -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message = "Aligning to table" + (argument.empty() ? "" : ": " + argument);

  goalHandle->publish_feedback(feedback);

  // -------------------- SERVICE CALL --------------------
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node->get_logger(), "Table Align service not available.");
    return executor.authorize("align", argument, "failure", "Service unavailable");
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_future = client->async_send_request(request);

  bool service_success = false;
  std::string response_message = "Timeout";

  // Alignment takes max 15s in the table_align node, so 20s wait is safe.
  if (result_future.wait_for(std::chrono::seconds(20)) == std::future_status::ready) {
    auto response = result_future.get();
    service_success = response->success;
    response_message = response->message;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Table Align service response timed out.");
  }

  // -------------------- AUTHORIZE --------------------
  return executor.authorize("align", argument,
                            service_success ? "success" : "failure",
                            service_success ? "Alignment completed" : "Alignment failed");
}
