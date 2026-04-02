#include "executor_tools/Take.hpp"

using namespace std::chrono_literals;

Take::Take(const rclcpp::Node::SharedPtr &node)
    : node(node), executor(node) {
  take_client_ = node->create_client<TakeSrv>("/task/take");
}

bool Take::run(const std::string &object_name,
               const std::shared_ptr<GoalHandle> &goalHandle) {

  // -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message = "Taking " + object_name;
  goalHandle->publish_feedback(feedback);

  // -------------------- WAIT FOR SERVICE --------------------
  if (!take_client_->wait_for_service(2s)) {
    RCLCPP_ERROR(node->get_logger(), "Take service unavailable");
    return executor.authorize("Take", object_name, "failure",
                              "Take service unavailable");
  }

  // -------------------- REQUEST --------------------
  auto request = std::make_shared<TakeSrv::Request>();
  request->object_name = object_name;

  auto future = take_client_->async_send_request(request);

  // -------------------- WAIT RESPONSE --------------------
  if (future.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "Take service timeout");
    return executor.authorize("Take", object_name, "failure",
                              "Take service timeout");
  }

  auto response = future.get();
  bool take_completed = response->success;

  return executor.authorize("Take", object_name,
                            take_completed ? "success" : "failure",
                            std::string("Taking ") + object_name);
}