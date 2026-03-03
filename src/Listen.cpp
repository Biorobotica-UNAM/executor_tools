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
  // -------------------- FEEDBACK --------------------
  auto feedback =
    std::make_shared<ExecuteTask::Feedback>();

  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message =
    "Listening with grammar: " + grammar;

  goalHandle->publish_feedback(feedback);

  // -------------------- HRI LISTEN --------------------
  std::string recognized = hri->Listen(grammar);

  if (recognized.empty())
  {
    recognized = "no_input";
  }

  knowledge->setLastRecognized(recognized); // Guardamos el dato

  // -------------------- AUTHORIZE --------------------
  return executor.authorize(
    "listen",
    grammar,
    recognized,
    "Recognized: " + recognized
  );
}

