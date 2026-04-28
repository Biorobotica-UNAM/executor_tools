#include "executor_tools/MemorizeFace.hpp"
#include "vision_tools/Vision.hpp"

MemorizeFace::MemorizeFace(const rclcpp::Node::SharedPtr &node,
                           const std::shared_ptr<Vision> &vision,
                           const std::shared_ptr<Knowledge> &knowledge)
    : node(node), vision(vision), knowledge(knowledge), executor(node) {}

bool MemorizeFace::run(const std::string &person_name,
                      const std::shared_ptr<GoalHandle> &goalHandle) {

  // 1. -------------------- RESOLUCIÓN DE NOMBRE --------------------
  if (person_name.empty() || person_name == "unknown") {
    return executor.authorize("memorize_face", person_name, "failure", "Invalid name");
  }

  // Traducción: name_1 -> Chris
  std::string real_name = knowledge->getNameFromLabel(person_name);
  
  if (real_name == "./." || real_name.empty()) {
      real_name = person_name; 
  }

  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.message = "MemorizeFace: Capturing face for " + real_name;
  goalHandle->publish_feedback(feedback);

  // 2. -------------------- VERIFICACIÓN --------------------
  bool is_someone_there = vision->IdentifyPerson("unknown"); 

  if (!is_someone_there) {
    RCLCPP_WARN(node->get_logger(), "MemorizeFace: No face in FOV.");
    return executor.authorize("memorize_face", person_name, "failure", "No face detected");
  }

  // 3. -------------------- REGISTRO --------------------
  bool registered = false;
  try {
      registered = vision->RegisterPerson(real_name);
  } catch (const std::exception &e) {
      RCLCPP_ERROR(node->get_logger(), "Vision service crashed: %s", e.what());
      return executor.authorize("memorize_face", person_name, "failure", "Service exception");
  }

  // 4. -------------------- RESULTADO --------------------
  if (registered) {
    RCLCPP_INFO(node->get_logger(), "MemorizeFace: %s saved.", real_name.c_str());
    return executor.authorize("memorize_face", person_name, "success", "Face registered as " + real_name);
  } else {
    return executor.authorize("memorize_face", person_name, "failure", "Registration failed");
  }
}