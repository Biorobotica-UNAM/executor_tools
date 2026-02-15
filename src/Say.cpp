#include "executor_tools/Say.hpp"

Say::Say(const rclcpp::Node::SharedPtr& node_ptr,
         const std::shared_ptr<HRI>& hri_ptr)
: node(node_ptr),
  hri(hri_ptr),
  executor(node_ptr)
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

  bool ok = hri->Say(text);

  std::string worker_result =
    ok ? "uttered: " + text : "tts_failed";

  return executor.authorize(
    "say",
    text,
    worker_result,
    "TTS executed");
}

