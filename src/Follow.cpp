#include "executor_tools/Follow.hpp"

Follow::Follow(const rclcpp::Node::SharedPtr &node)
    : node(node), executor(node) {
  // Inicializamos el cliente apuntando al nodo de tracking
  client = node->create_client<std_srvs::srv::SetBool>("/hri/human_tracking/enable");
}

bool Follow::run(const std::string &arg,
               const std::shared_ptr<GoalHandle> &goalHandle) {

  // -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message = "Activating human tracking...";

  goalHandle->publish_feedback(feedback);

  // -------------------- SERVICE CALL --------------------

  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node->get_logger(), "Tracking service not available.");
    return executor.authorize("follow", arg, "failure", "Service unavailable");
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  // Si arg es "stop", apagamos; de lo contrario encendemos
  request->data = (arg != "stop"); 

  auto result_future = client->async_send_request(request);

  bool service_success = false;
  if (result_future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
    auto response = result_future.get();
    service_success = response->success;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service response timed out.");
  }

  // -------------------- AUTHORIZE --------------------

  return executor.authorize("follow", arg,
                            service_success ? "success" : "failure",
                            service_success ? "Tracking active" : "Tracking failed");
}