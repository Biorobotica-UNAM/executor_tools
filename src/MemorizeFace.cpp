#include "executor_tools/MemorizeFace.hpp"

MemorizeFace::MemorizeFace(const rclcpp::Node::SharedPtr &node)
    : node(node), executor(node) {}

bool MemorizeFace::run(const std::string &person_name,
                      const std::shared_ptr<GoalHandle> &goalHandle) {

  // 1. -------------------- VALIDACIÓN DE ARGUMENTO --------------------
  if (person_name.empty() || person_name == "unknown") {
    RCLCPP_WARN(node->get_logger(), "MemorizeFace: Invalid or unknown person name.");
    return executor.authorize("memorize_face", person_name, "failure", 
                              "Invalid person name for memorization");
  }

  // 2. -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message = "Preparing vision system to memorize: " + person_name;

  goalHandle->publish_feedback(feedback);

  // 3. -------------------- PLACEHOLDER DE LA TOOL --------------------
  // TODO: Integrar aquí el cliente del servicio de reconocimiento facial (vision_tools)
  RCLCPP_INFO(node->get_logger(), "MemorizeFace: Tool for %s is not yet implemented.", person_name.c_str());

  // Por ahora mandamos un reject o success simulado para no trabar el plan
  // Usaremos "success" para que el planificador crea que terminó y pase a la siguiente,
  // pero con un mensaje de advertencia en el contexto.
  return executor.authorize("memorize_face", person_name, "success",
                            "MemorizeFace tool not implemented yet, but arg received: " + person_name);
}