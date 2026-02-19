#include "executor_tools/GoTo.hpp"

GoTo::GoTo(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Navigation> &navigation)
    : node(node), navigation(navigation), executor(node) {}

bool GoTo::run(const std::string &location_name,
               const std::shared_ptr<GoalHandle> &goalHandle) {

  // -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message = "Going to " + location_name + " location";

  goalHandle->publish_feedback(feedback);

  // -------------------- NAVIGATION GOTO --------------------

  bool navigation_completed = navigation->GoTo(location_name);

  return executor.authorize(
      "GoTo", location_name, navigation_completed ? "success" : "failure",
      std::string("Going to ") + location_name ));
}
