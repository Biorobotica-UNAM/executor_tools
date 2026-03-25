#include "executor_tools/Find.hpp"

using namespace std::chrono_literals;

Find::Find(const rclcpp::Node::SharedPtr &node)
    : node(node), executor(node) {
  // Creamos el cliente que apunta al servicio del PersonFinderNode
  client = node->create_client<faces_interfaces::srv::FaceRecog>("/tasks/find_person");
}

bool Find::run(const std::string &person_name,
               const std::shared_ptr<GoalHandle> &goalHandle) {

  // 1. -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.message = "Find: Starting active search for " + (person_name.empty() ? "anyone" : person_name);
  goalHandle->publish_feedback(feedback);

  // 2. -------------------- ESPERA DEL SERVICIO --------------------
  if (!client->wait_for_service(2s)) {
    RCLCPP_ERROR(node->get_logger(), "Find: PersonFinderNode service not available!");
    return executor.authorize("Find", person_name, "failure", "FSM Service unavailable");
  }

  // 3. -------------------- PETICIÓN (REQUEST) --------------------
  auto request = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
  request->name = person_name; 

  auto future = client->async_send_request(request);

  // 4. -------------------- ESPERA SÍNCRONA (TIMEOUT) --------------------
  if (rclcpp::spin_until_future_complete(node, future, 30s) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Find: Service call failed or timed out.");
    return executor.authorize("Find", person_name, "failure", "Search timed out");
  }

  auto response = future.get();

  // 5. -------------------- RESULTADO FINAL --------------------
  if (response->success) {
    RCLCPP_INFO(node->get_logger(), "Find: Target located!");
    return executor.authorize("Find", person_name, "success", "Person found by FSM");
  } else {
    RCLCPP_WARN(node->get_logger(), "Find: Target not found after full scan.");
    return executor.authorize("Find", person_name, "failure", "Person not detected");
  }
}