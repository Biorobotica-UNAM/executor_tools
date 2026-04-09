#include "executor_tools/Align.hpp"

using namespace std::chrono_literals;

Align::Align(const rclcpp::Node::SharedPtr &node)
    : node(node), executor(node) { 
  align_client_ = node->create_client<AlignSrv>("vision/align");
}

bool Align::run(const std::string &target_name,
                const std::shared_ptr<GoalHandle> &goalHandle) {

  // -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message = "Aligning with " + target_name;
  goalHandle->publish_feedback(feedback);

  // -------------------- WAIT FOR SERVICE --------------------
  if (!align_client_->wait_for_service(2s)) {
    RCLCPP_ERROR(node->get_logger(), "Align service unavailable");
    return executor.authorize("Align", target_name, "failure",
                              "Align service unavailable");
  }

  // -------------------- REQUEST --------------------
  auto request = std::make_shared<AlignSrv::Request>();
  request->target_name = target_name;

  auto future = align_client_->async_send_request(request);

  // -------------------- WAIT RESPONSE --------------------
  if (future.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "Align service timeout");
    return executor.authorize("Align", target_name, "failure",
                              "Align service timeout");
  }

  auto response = future.get();
  bool align_completed = response->success;

  return executor.authorize("Align", target_name,
                            align_completed ? "success" : "failure",
                            std::string("Aligning with ") + target_name);
}