#include "executor_tools/Say.hpp"

#include "hri_tools/HRI.hpp"

Say::Say(const rclcpp::Node::SharedPtr& node)
: node_(node),
  auth_(node)
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
  feedback->feedback.message = "Saying: " + text;

  goal_handle->publish_feedback(feedback);

  // Aquí iría hri_.say(text);
  bool local_result = true;

  bool authorized =
    auth_.request("say",
                  text,
                  local_result,
                  "TTS executed");

  return authorized;
}

