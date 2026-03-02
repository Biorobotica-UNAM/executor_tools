#include "executor_tools/Knowledge.hpp"
#include "executor_tools/Save.hpp"

Save::Save(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Knowledge> &knowledge)
    : node(node), knowledge(knowledge), executor(node) {}

bool Save::run(const std::string &argument,
                const std::shared_ptr<GoalHandle> &goalHandle) {

  // 1. -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message = "Calling the Knowledge tool...";

  goalHandle->publish_feedback(feedback);

  // 2. -------------------- CALL KNOWLEDGE TOOL --------------------
  bool success = knowledge->saveInfo(argument); 

  // 3. -------------------- AUTHORIZATION --------------------
  return executor.authorize("Save", argument,
                            success ? "success" : "failure",
                            success ? "Data successfully saved in Knowledge" 
                                    : "Knowledge tool failed to save data");
}