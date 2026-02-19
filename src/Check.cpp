#include "executor_tools/Check.hpp"

Check::Check(const rclcpp::Node::SharedPtr &node,
             const std::shared_ptr<Hardware> &hardware)
    : node(node), hardware(hardware), executor(node) {}

bool Check::run(const std::string &argument,
                const std::shared_ptr<GoalHandle> &goalHandle) {

  // -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message = "Checking for " + argument;

  goalHandle->publish_feedback(feedback);

  // -------------------- HARDWARE CHECK --------------------
  bool door_open = hardware->CheckDoor();

  return executor.authorize("Check", argument,
                            door_open ? "success" : "failure",
                            std::string("Checking ") + argument ));
}
