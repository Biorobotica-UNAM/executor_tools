#include "executor_tools/Listen.hpp"

Listen::Listen(const rclcpp::Node::SharedPtr& node,
               const std::shared_ptr<HRI>& hri,
               const std::shared_ptr<Knowledge>& knowledge)
: knowledge(knowledge),
  node(node),
  hri(hri), 
  executor(node)
{
}

bool Listen::run(
  const std::string& grammar,
  const std::shared_ptr<GoalHandle>& goalHandle)
{
  // 1. Verificación de seguridad: ¿Sigue vivo el cliente?
  if (!goalHandle || !goalHandle->is_active()) {
    RCLCPP_WARN(node->get_logger(), "Listen Action: GoalHandle is null or inactive.");
    return false;
  }

  std::string grammar_file = grammar + ".json";

  try {
    // 2. Feedback con verificación
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.message = "Listening with grammar: " + grammar_file;
    
    if (goalHandle->is_active()) {
        goalHandle->publish_feedback(feedback);
    }

    // 3. Llamada al HRI con validación de nulidad
    if (!hri) {
        RCLCPP_ERROR(node->get_logger(), "HRI pointer is null in Listen Tool!");
        return false;
    }

    std::string recognized = hri->Listen(grammar_file);

    // Si hri->Listen truena internamente, el try-catch del Worker lo atrapará,
    // pero aquí manejamos el caso de respuesta vacía o error lógico.
    if (recognized.empty() || recognized == "error") 
    {
      RCLCPP_WARN(node->get_logger(), "HRI recognized nothing or returned error.");
      recognized = "no_input";
    }

    // 4. Guardar en Knowledge (Verificando que exista el objeto)
    if (knowledge) {
        knowledge->setLastRecognized(recognized);
    }

    // 5. Authorize (El último paso)
    return executor.authorize(
      "listen",
      grammar,
      recognized,
      "Recognized: " + recognized
    );

  } catch (const std::exception &e) {
      RCLCPP_ERROR(node->get_logger(), "Exception in Listen::run: %s", e.what());
      return false;
  }
}