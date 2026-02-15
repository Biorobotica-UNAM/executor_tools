#include "executor_tools/Listen.hpp"

Listen::Listen(const rclcpp::Node::SharedPtr& node_ptr,
               const std::shared_ptr<HRI>& hri_ptr)
: node(node_ptr),
  hri(hri_ptr),
  executor(node_ptr)
{
}

bool Listen::run(
  const std::string& grammar,
  const std::shared_ptr<GoalHandle>& goal_handle)
{
  auto feedback =
    std::make_shared<ExecuteTask::Feedback>();

  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message =
    "Listening with grammar: " + grammar;

  goal_handle->publish_feedback(feedback);

  std::string recognized = hri->Listen(grammar);

  std::string worker_result =
    recognized.empty() ? "no_input" : recognized;

  return executor.authorize(
    "listen",
    grammar,
    worker_result,
    "Recognized: " + worker_result);
}

