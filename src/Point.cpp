#include "executor_tools/Point.hpp"
#include "tmc_control_msgs/action/gripper_apply_effort.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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
  else if(target_name == "bag")
  {
    RCLCPP_INFO(node->get_logger(), "Point tool: Sending Grasp effort -0.1 to gripper...");
    
    using GripperApplyEffort = tmc_control_msgs::action::GripperApplyEffort;
    auto client = rclcpp_action::create_client<GripperApplyEffort>(node, "/gripper_controller/grasp");

    if (!client->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node->get_logger(), "Action server /gripper_controller/grasp not available");
      motion_completed = false;
    } else {
      auto goal_msg = GripperApplyEffort::Goal();
      goal_msg.effort = -0.1;
      goal_msg.do_control_stop = true;

      // Enviamos el goal y esperamos el resultado de forma síncrona para el worker
      auto send_goal_options = rclcpp_action::Client<GripperApplyEffort>::SendGoalOptions();
      auto goal_handle_future = client->async_send_goal(goal_msg, send_goal_options);
      
      // Bloqueamos hasta que el servidor acepte la meta
      if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
        motion_completed = false;
      } else {
        auto goal_handle = goal_handle_future.get();
        auto result_future = client->async_get_result(goal_handle);
        
        // Bloqueamos hasta que el gripper termine de aplicar el esfuerzo
        if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
          motion_completed = true;
          RCLCPP_INFO(node->get_logger(), "Grasp effort applied successfully.");
        } else {
          motion_completed = false;
        }
      }
    }
  }
  else if(target_name == "nobag")
  {

  }

  // -------------------- AUTHORIZE --------------------

  return executor.authorize("Point", target_name,
                            motion_completed ? "success" : "failure",
                            motion_completed ? "Pointing motion finished" : "Pointing motion failed");
}