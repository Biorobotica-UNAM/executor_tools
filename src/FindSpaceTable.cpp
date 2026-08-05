#include "executor_tools/FindSpace.hpp"

#include <chrono>
#include <future>
#include <string>

using namespace std::chrono_literals;

void FindSpace::configureTablePreferenceForHeldObject() {
  const json left_object = knowledge_->get_world_state("robot", "", "left_hand");
  const json right_object =
      knowledge_->get_world_state("robot", "", "right_hand");

  if (!left_object.is_string() || !right_object.is_string()) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] no se pudo leer ocupacion de manos; se conserva "
                "preferred_x_m de launch");
    return;
  }

  const bool left_occupied = left_object.get<std::string>() != "nothing";
  const bool right_occupied = right_object.get<std::string>() != "nothing";

  if (!left_occupied && !right_occupied) {
    RCLCPP_INFO(node_->get_logger(),
                "[FindSpace] ninguna mano ocupada; se conserva preferred_x_m "
                "de launch para mesa");
    return;
  }

  const bool prefer_right = right_occupied || !left_occupied;
  const double preferred_x = prefer_right ? 0.35 : -0.35;
  const std::string preferred_hand = prefer_right ? "right_hand" : "left_hand";

  rclcpp::AsyncParametersClient parameter_client(
      node_, "/vision/table_drop_tf_node");
  if (!parameter_client.wait_for_service(500ms)) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] parametros de /vision/table_drop_tf_node no "
                "disponibles; se conserva preferred_x_m de launch");
    return;
  }

  auto future = parameter_client.set_parameters(
      {rclcpp::Parameter("preferred_x_m", preferred_x)});
  if (future.wait_for(1s) != std::future_status::ready) {
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] timeout ajustando preferred_x_m para mesa; se "
                "conserva el valor actual");
    return;
  }

  const auto results = future.get();
  if (results.empty() || !results.front().successful) {
    const std::string reason =
        results.empty() ? "empty_result" : results.front().reason;
    RCLCPP_WARN(node_->get_logger(),
                "[FindSpace] fallo ajustando preferred_x_m para mesa: %s",
                reason.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[FindSpace] preferred_x_m de mesa ajustado a %.2f por %s "
              "ocupada (left='%s' right='%s')",
              preferred_x, preferred_hand.c_str(),
              left_object.get<std::string>().c_str(),
              right_object.get<std::string>().c_str());
}
