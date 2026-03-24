#include "executor_tools/Wait.hpp"

Wait::Wait(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Vision> &vision)
    : node(node), vision(vision), executor(node) {}

bool Wait::run(const std::string &argument,
               const std::shared_ptr<GoalHandle> &goalHandle) {

  // -------------------- FEEDBACK INICIAL --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.1f;
  feedback->feedback.message = "Wait: Initializing vision system...";
  goalHandle->publish_feedback(feedback);

  RCLCPP_INFO(node->get_logger(), "Wait tool started. Using Vision interface.");

  // -------------------- BUCLE DE ESPERA --------------------
  rclcpp::WallRate rate(5); // Bajamos un poco la frecuencia para no saturar los servicios
  bool person_found = false;

  while (rclcpp::ok() && !person_found) {
    // 1. Verificamos si el Supervisor canceló la tarea
    if (goalHandle->is_canceling()) {
      return executor.authorize("wait", argument, "failure", "Wait action canceled");
    }

    // 2. Usamos la interfaz de Vision para detectar a la persona
    person_found = vision->DetectPerson("unknown");

    if (!person_found) {
        feedback->feedback.message = "Still waiting for a person to appear...";
        goalHandle->publish_feedback(feedback);
        rate.sleep();
    }
  }

  // Si salimos del bucle es porque person_found es true
  RCLCPP_INFO(node->get_logger(), "Person detected via Vision Interface!");

  return executor.authorize("wait", argument, "success", "Person detected, wait finished");
}