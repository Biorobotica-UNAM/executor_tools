#include "executor_tools/Listen.hpp"

Listen::Listen(const rclcpp::Node::SharedPtr& node)
: node_(node),
  auth_(node)
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

  // Simulación de resultado real
  bool local_result = true;
  std::string heard = "coke";

  std::string context =
    "Heard: " + heard;

  bool authorized =
    auth_.request("listen",
                  grammar,
                  local_result,
                  context);

  return authorized;
}

