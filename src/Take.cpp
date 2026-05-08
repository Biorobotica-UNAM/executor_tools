#include "executor_tools/Take.hpp"
#include <chrono>
#include <future> 

using namespace std::chrono_literals;

Take::Take(const rclcpp::Node::SharedPtr &node,
           const std::shared_ptr<Knowledge> &knowledge)
    : node(node), knowledge(knowledge), executor(node) {}

bool Take::run(const std::string &object_name,
               const std::shared_ptr<GoalHandle> &goalHandle) {

  // 1. FEEDBACK
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.message = "Iniciando manipulación para: " + object_name;
  goalHandle->publish_feedback(feedback);

  // 2. OBTENER POSE COMPLETA (PoseStamped) DE LA RAM
  // Ahora el Knowledge devuelve la pose con la orientación real de TF2
  geometry_msgs::msg::PoseStamped target_pose = knowledge->getObjectPose(object_name);
  
  // Verificación básica: Si la pose está vacía (x e y en 0)
  if (target_pose.pose.position.x == 0 && target_pose.pose.position.y == 0) {
      RCLCPP_WARN(node->get_logger(), "Take: No se encontró la pose de %s en RAM", object_name.c_str());
      return executor.authorize("take", object_name, "failure", "Object pose not found in RAM");
  }

  // OPCIONAL: Si el brazo requiere una orientación específica por diseño (ej. siempre desde arriba)
  // puedes sobreescribir el quaternion aquí, pero lo ideal es usar el que ya calculó la visión.
  /*
  target_pose.pose.orientation.x = 0.0; 
  target_pose.pose.orientation.y = -0.7071; 
  target_pose.pose.orientation.z = 0.0; 
  target_pose.pose.orientation.w = 0.7071;
  */

  // --- LÓGICA DE SELECCIÓN DE BRAZO ---
  // Puedes dejarlo en "left" o decidir según la coordenada 'y' del objeto
  std::string selected_arm = (target_pose.pose.position.y > 0) ? "left" : "right";
  RCLCPP_INFO(node->get_logger(), "Take: Seleccionando brazo %s para alcanzar objeto", selected_arm.c_str());

  // 3. CONFIGURAR CLIENTE Y REQUEST
  auto client = node->create_client<take_interfaces::srv::Take>("/take");

  if (!client->wait_for_service(3s)) {
      return executor.authorize("take", object_name, "failure", "Service /take not available");
  }

  auto request = std::make_shared<take_interfaces::srv::Take::Request>();
  request->arm = selected_arm; 
  request->target_pose = target_pose; // Enviamos la PoseStamped completa

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
}