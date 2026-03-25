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
  feedback->feedback.message = "Wait: Starting Active Search (FSM)...";
  goalHandle->publish_feedback(feedback);

  RCLCPP_INFO(node->get_logger(), "Wait tool started. Invoking PersonFinder FSM.");

  // -------------------- LÓGICA DE EJECUCIÓN --------------------
  bool person_found = false;

  try {
      if (goalHandle->is_canceling()) {
          return executor.authorize("wait", argument, "failure", "Wait canceled before start");
      }

      std::string target = argument.empty() ? "unknown" : argument;
      
      feedback->feedback.message = "Justina is scanning the room for: " + target;
      goalHandle->publish_feedback(feedback);

      person_found = vision->DetectPerson(target);

  } catch (const std::exception &e) {
      RCLCPP_ERROR(node->get_logger(), "Wait: FSM Service Error: %s", e.what());
      return executor.authorize("wait", argument, "failure", "FSM Service unavailable");
  }

  // -------------------- RESULTADO FINAL --------------------
  if (person_found) {
      feedback->feedback.message = "Target located successfully!";
      goalHandle->publish_feedback(feedback);
      RCLCPP_INFO(node->get_logger(), "Wait: Person found by FSM.");
      return executor.authorize("wait", argument, "success", "Person located, scan finished");
  } else {
      RCLCPP_WARN(node->get_logger(), "Wait: FSM finished without finding anyone.");
      return executor.authorize("wait", argument, "failure", "No person detected after full scan");
  }
}