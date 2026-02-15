#include "executor_tools/Say.hpp"

Say::Say(const rclcpp::Node::SharedPtr& node)
: node(node),
  hri(std::make_shared<HRI>(node)),
  executor(node)
{
}

bool Say::run(
  const std::string& text,
  const std::shared_ptr<GoalHandle>& goal_handle)
{
  auto feedback =
    std::make_shared<ExecuteTask::Feedback>();

  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message =
    "Saying: " + text;

  goal_handle->publish_feedback(feedback);

  hri->Say(text);

  std::string worker_result = "uttered: " + text;

  return executor.authorize(
    "say",
    text,
    worker_result,
    "TTS executed");
}

