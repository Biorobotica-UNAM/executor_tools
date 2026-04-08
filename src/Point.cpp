#include "executor_tools/Point.hpp"

Point::Point(const rclcpp::Node::SharedPtr &node,
             const std::shared_ptr<Hardware> &hardware)
    : node(node), hardware(hardware), executor(node) {}

bool Point::run(const std::string &target_name,
               const std::shared_ptr<GoalHandle> &goalHandle) {

  // -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  bool motion_completed = false;
  feedback->feedback.message = "Executing pointing motion to: " + target_name;
  goalHandle->publish_feedback(feedback);

  RCLCPP_INFO(node->get_logger(), "Point tool: Aligning and pointing to target. Target: %s", target_name.c_str());

  // -------------------- HARDWARE POINTING --------------------
  
  if(target_name == "enable")
  {
    motion_completed = hardware->PointingMotionEnable();
  }
  else if(target_name == "disable")
  {
    motion_completed = hardware->PointingMotionDisable();
  }

  // -------------------- AUTHORIZE --------------------

  return executor.authorize("Point", target_name,
                            motion_completed ? "success" : "failure",
                            motion_completed ? "Pointing motion finished" : "Pointing motion failed");
}