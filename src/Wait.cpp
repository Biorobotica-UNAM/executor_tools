#include "executor_tools/Wait.hpp"

Wait::Wait(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Vision> &vision)
    : node(node), vision(vision), executor(node) {}

bool Wait::run(const std::string &argument,
               const std::shared_ptr<GoalHandle> &goalHandle)
    {

        // -------------------- FEEDBACK --------------------
        auto feedback = std::make_shared<ExecuteTask::Feedback>();
        feedback->feedback.step_id = 1;
        feedback->feedback.progress = 0.5f;
        feedback->feedback.message = "Waiting to " + argument;

        goalHandle->publish_feedback(feedback);

        // -------------------- VISION wait --------------------

        bool wait_completed = vision->DetectPerson(argument);

        return executor.authorize("Wait", argument,
                                  wait_completed ? "success" : "failure",
                                  std::string("Waiting to ") + argument);
    }    
    
    
