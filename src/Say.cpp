#include "executor_tools/Say.hpp"

Say::Say(const rclcpp::Node::SharedPtr& node,
         const std::shared_ptr<HRI>& hri)
: node(node),
  hri(hri),
  executor(node)
{
}

bool Say::run(
  const std::string& text,
  const std::shared_ptr<GoalHandle>& goalHandle)
{
  // -------------------- FEEDBACK --------------------
  auto feedback =
    std::make_shared<ExecuteTask::Feedback>();

  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message =
    "Saying: " + text;

  goalHandle->publish_feedback(feedback);

  // -------------------- HRI SAY --------------------
  bool success = hri->Say(text);

  std::string result =
    success ? "uttered: " + text : "tts_failed";

  // -------------------- AUTHORIZE --------------------
  return executor.authorize(
    "say",
    text,
    result,
    success ? "TTS executed" : "TTS failed"
  );
}

