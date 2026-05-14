#include "executor_tools/Saveloc.hpp"
#include <chrono>

using namespace std::chrono_literals;

Saveloc::Saveloc(const rclcpp::Node::SharedPtr &node)
    : node(node), executor(node) {}

bool Saveloc::run(const std::string &location_name,
                 const std::shared_ptr<GoalHandle> &goalHandle) {

  // 1. Verificación de seguridad
  if (!goalHandle || !goalHandle->is_active()) return false;

  // -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.message = "Saving current location as: " + location_name;
  goalHandle->publish_feedback(feedback);

  // -------------------- SERVICE CALL (Auxiliar) --------------------
  // Creamos nodo auxiliar para no bloquear el spin principal
  auto aux_node = rclcpp::Node::make_shared("saveloc_aux_node");
  auto client = aux_node->create_client<known_locations_interfaces::srv::LocationsServer>("/locations_manager/add");

  rclcpp::executors::SingleThreadedExecutor aux_exec;
  aux_exec.add_node(aux_node);

  // Esperar al servicio (máximo 3 segundos)
  if (!client->wait_for_service(3s)) {    
    RCLCPP_ERROR(node->get_logger(), "Saveloc: Service /locations_manager/add not available");
    return executor.authorize("saveloc", location_name, "failure", "Service not available");
  }

  // Preparamos el request con el argumento que llegó
  auto request = std::make_shared<known_locations_interfaces::srv::LocationsServer::Request>();
  request->location_name = location_name; 

  auto future = client->async_send_request(request);

  // Spin hasta que responda o pasen 5s
  auto status = aux_exec.spin_until_future_complete(future, 5s);

  bool success = false;
  std::string result_msg;

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        success = response->success;
        // Quitamos la referencia a response->message
        result_msg = success ? "location_saved" : "save_rejected by service";
    }

  // -------------------- AUTHORIZE --------------------
  return executor.authorize("saveloc", location_name,
                            success ? "success" : "failure",
                            "Location " + location_name + " save attempt: " + result_msg);
}