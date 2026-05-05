#include "executor_tools/FindSeat.hpp"

FindSeat::FindSeat(const rclcpp::Node::SharedPtr& node,
                   const std::shared_ptr<Tasks>& tasks,
                   const std::shared_ptr<Knowledge>& knowledge)
: node(node), tasks(tasks), knowledge(knowledge), executor(node)
{}

bool FindSeat::run(const std::string& argument,
                   const std::shared_ptr<GoalHandle>& goalHandle)
{
    if (!goalHandle || !goalHandle->is_active()) return false;

    // 1. Feedback al Supervisor
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.message = "Buscando un asiento disponible...";
    goalHandle->publish_feedback(feedback);

    RCLCPP_INFO(node->get_logger(), "FindSeat: Starting search for empty seats.");

    bool success = false;
    std::string result_msg = "not_found: no_empty_seats";

    try {
        // 2. Llamada al servicio Trigger de Tasks
        auto response = tasks->FindSeat();
        success = response.success;

        if (success) {
            // El mensaje del servicio suele contener info del asiento (ej. "Seat at left")
            result_msg = "found: " + response.message;
            
            // Guardamos en RAM que encontramos un asiento
            knowledge->saveInRAM("last_seat_status", "available");
            
            RCLCPP_INFO(node->get_logger(), "FindSeat SUCCESS: %s", response.message.c_str());
        } else {
            result_msg = "not_found: " + response.message;
            RCLCPP_WARN(node->get_logger(), "FindSeat FAILED: %s", response.message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "FindSeat CRASH: %s", e.what());
        result_msg = "not_found: internal_error";
    }

    // 3. Autorización final
    return executor.authorize(
        "find_seat",
        argument,
        result_msg,
        success ? "Asiento localizado" : "No se encontró asiento libre"
    );
}