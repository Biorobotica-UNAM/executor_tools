#include "executor_tools/Wait.hpp"
#include <chrono>

Wait::Wait(const rclcpp::Node::SharedPtr &node)
    : node(node), executor(node), person_detected_(false) {}

void Wait::detection_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  this->person_detected_ = msg->data;
}

bool Wait::run(const std::string &argument,
               const std::shared_ptr<GoalHandle> &goalHandle) {

  person_detected_ = false;

  // 1. Crear la suscripción
  auto sub = node->create_subscription<std_msgs::msg::Bool>(
      "/human_detection/human_detected", 10,
      std::bind(&Wait::detection_callback, this, std::placeholders::_1));

  // 2. VALIDACIÓN
  rclcpp::sleep_for(std::chrono::milliseconds(500)); 
  
  if (node->count_publishers("/human_detection/human_detected") == 0) {
    RCLCPP_ERROR(node->get_logger(), "Wait: Vision node (YOLO) is NOT running!");
    return executor.authorize("wait", argument, "failure", 
                              "Vision system is offline (No publishers found)");
  }

  // -------------------- FEEDBACK INICIAL --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.1f;
  feedback->feedback.message = "Wait: Camera online. Waiting for person...";
  goalHandle->publish_feedback(feedback);

  // -------------------- BUCLE DE ESPERA --------------------
  rclcpp::WallRate rate(10); 
  
  RCLCPP_INFO(node->get_logger(), "Wait tool started. Tracking publishers...");

  while (rclcpp::ok() && !person_detected_) {
    if (goalHandle->is_canceling()) {
      return executor.authorize("wait", argument, "failure", "Wait action canceled");
    }

    // Validación extra: Si el nodo de visión se muere mientras esperamos
    if (node->count_publishers("/human_detection/human_detected") == 0) {
        return executor.authorize("wait", argument, "failure", "Vision system went offline during wait");
    }

    feedback->feedback.message = "Still waiting for a person...";
    goalHandle->publish_feedback(feedback);

    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Person detected! Ending wait action.");
  return executor.authorize("wait", argument, "success", "Person detected, wait finished");
}