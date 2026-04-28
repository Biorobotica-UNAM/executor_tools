#include "executor_tools/Knowledge.hpp"
#include "executor_tools/Save.hpp"
#include <vector>
#include <sstream>

Save::Save(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Knowledge> &knowledge)
    : node(node), knowledge(knowledge), executor(node) {}

bool Save::run(const std::string &argument,
                const std::shared_ptr<GoalHandle> &goalHandle) {

  // 1. -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message = "Calling the Knowledge tool...";

  goalHandle->publish_feedback(feedback);

  // 2. -------------------- CALL KNOWLEDGE TOOL --------------------
  
  // Obtenemos lo último reconocido (ej: "coffee milk")
  std::string recognized = knowledge->getLastRecognized();
  std::string final_value = recognized;

  // Lógica especial para órdenes (Restaurant)
  // Si el argumento contiene "order", cambiamos el espacio por " and "
  if (argument.find("order") != std::string::npos) {
      size_t space_pos = recognized.find(' ');
      if (space_pos != std::string::npos) {
          // Reemplaza "coffee milk" -> "coffee and milk"
          final_value.replace(space_pos, 1, " and ");
          RCLCPP_INFO(node->get_logger(), "Save: Formatted order with 'and': %s", final_value.c_str());
      }
  }

  // Usamos updateFromService para persistir en RAM y Disco
  bool success = knowledge->updateFromService(argument, final_value);

  // 3. -------------------- AUTHORIZATION --------------------
  return executor.authorize("Save", argument,
                            success ? "success" : "failure",
                            success ? ("Data saved as: " + final_value) 
                                    : "Knowledge tool failed to save data");
}