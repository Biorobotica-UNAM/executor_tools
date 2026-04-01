#include "executor_tools/MemorizeFace.hpp"
#include "vision_tools/Vision.hpp"

MemorizeFace::MemorizeFace(const rclcpp::Node::SharedPtr &node,
                           const std::shared_ptr<Vision> &vision)
    : node(node), vision(vision), executor(node) {}

bool MemorizeFace::run(const std::string &person_name,
                      const std::shared_ptr<GoalHandle> &goalHandle) {

  // 1. -------------------- VALIDACIÓN --------------------
  if (person_name.empty() || person_name == "unknown") {
    return executor.authorize("memorize_face", person_name, "failure", "Invalid name");
  }

  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.message = "MemorizeFace: Capturing face for " + person_name;
  goalHandle->publish_feedback(feedback);

  RCLCPP_INFO(node->get_logger(), "MemorizeFace: Registering person in current position.");

  // 2. -------------------- VERIFICACIÓN RÁPIDA --------------------
  
  bool is_someone_there = vision->IdentifyPerson("unknown"); 

  if (!is_someone_there) {
    RCLCPP_WARN(node->get_logger(), "MemorizeFace: No face detected in current FOV.");
    return executor.authorize("memorize_face", person_name, "failure", "No face in front of Justina");
  }

  // 3. -------------------- REGISTRO --------------------
  bool registered = false;
  try {
      // Directo al grano: guardar lo que estamos viendo
      registered = vision->RegisterPerson(person_name);
  } catch (const std::exception &e) {
      return executor.authorize("memorize_face", person_name, "failure", "Service error");
  }

  // 4. -------------------- RESULTADO --------------------
  if (registered) {
    RCLCPP_INFO(node->get_logger(), "MemorizeFace: %s saved.", person_name.c_str());
    return executor.authorize("memorize_face", person_name, "success", "Face registered");
  } else {
    return executor.authorize("memorize_face", person_name, "failure", "Capture failed");
  }
}