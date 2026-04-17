#include "executor_tools/Take.hpp"

using namespace std::chrono_literals;

Take::Take(const rclcpp::Node::SharedPtr &node)
    : node(node), executor(node) {
  // Cambia "/tasks/take_fsm" por el nombre real de tu servicio de la FSM
  client = node->create_client<std_srvs::srv::Trigger>("/tasks/take_fsm");
}

bool Take::run(const std::string &argument,
               const std::shared_ptr<GoalHandle> &goalHandle) {

  // 1. Feedback al Supervisor
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.message = "Executing Take FSM: " + argument;
  goalHandle->publish_feedback(feedback);

  // 2. Verificar disponibilidad del servicio
  if (!client->wait_for_service(2s)) {
    RCLCPP_ERROR(node->get_logger(), "Take FSM service not available.");
    return executor.authorize("take", argument, "failure", "Service unreachable");
  }

  // 3. Llamada asíncrona pero con espera controlada (bloqueante para el Worker)
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_future = client->async_send_request(request);

  bool success = false;
  std::string msg = "Timeout";

  // Espera de hasta 30 segundos (ajusta según lo que tarde tu FSM física)
  if (result_future.wait_for(300s) == std::future_status::ready) {
    auto response = result_future.get();
    success = response->success;
    msg = response->message;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Take FSM timed out.");
  }

  // 4. Reportar resultado final
  return executor.authorize("take", argument,
                            success ? "success" : "failure",
                            success ? "FSM Finished: " + msg : "FSM Failed: " + msg);
}








/*
#include "executor_tools/Take.hpp"
#include <chrono>
#include <future> 
 (!client->wait_for_service(3s)) {
      return executor.authorize("take", object_name, "failure", "Service /take not available");
  }

  auto request = std::make_shared<take_interfaces::srv::Take::Request>();
  request->arm = selected_arm; 
  request->target_pose = target_pose;

  // 4. LLAMADA ASÍNCRONA 
  auto result_future = client->async_send_request(request);
  if (result_future.wait_for(45s) == std::future_status::timeout) {
      return executor.authorize("take", object_name, "failure", "Manipulation service timeout");
  }

  // 5. OBTENER RESPUESTA Y AUTORIZAR
  auto response = result_future.get();

  return executor.authorize("take", object_name,
                            response->success ? "success" : "failure",
                            "Take result: " + response->message);
}>
using namespace std::chrono_literals;

Take::Take(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Knowledge> &knowledge)
    : node(node), knowledge(knowledge), executor(node) {}

bool Take::run(const std::string &object_name,
               const std::shared_ptr<GoalHandle> &goalHandle) {

  // 1. FEEDBACK
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.message = "Calling take service for: " + object_name;
  goalHandle->publish_feedback(feedback);

  // 2. OBTENER POSE DE LA RAM
  geometry_msgs::msg::PointStamped target_point = knowledge->getObjectPose(object_name);
  if (target_point.point.x == 0 && target_point.point.y == 0) {
      return executor.authorize("take", object_name, "failure", "Object pose not found in RAM");
  }

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header = target_point.header;
  target_pose.pose.position = target_point.point;
  // Cuaternión normalizado para rotación de 90° en Y (Mirando abajo)
  target_pose.pose.orientation.x = 0.0; 
  target_pose.pose.orientation.y = -0.7071; 
  target_pose.pose.orientation.z = 0.0; 
  target_pose.pose.orientation.w = 0.7071;

  // --- LÓGICA DE SELECCIÓN DE BRAZO ---
  std::string selected_arm;
  selected_arm = "left";

  // 3. CONFIGURAR CLIENTE Y REQUEST
  auto client = node->create_client<take_interfaces::srv::Take>("/take");

  if
  */