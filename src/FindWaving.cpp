#include "executor_tools/FindWaving.hpp"

FindWaving::FindWaving(const rclcpp::Node::SharedPtr& node,
                       const std::shared_ptr<Tasks>& tasks,
                       const std::shared_ptr<Knowledge>& knowledge)
: node(node), tasks(tasks), knowledge(knowledge), executor(node)
{}

bool FindWaving::run(const std::string& argument,
                     const std::shared_ptr<GoalHandle>& goalHandle)
{
    if (!goalHandle || !goalHandle->is_active()) return false;

    // 1. Feedback al Supervisor
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.message = "Buscando a alguien que agite la mano...";
    goalHandle->publish_feedback(feedback);

    RCLCPP_INFO(node->get_logger(), "FindWaving: Detecting waving gestures.");

    bool success = false;
    std::string result_msg = "not_found: no_one_waving";

    try {
        // 2. Llamada al servicio de visión para waving
        auto response = tasks->FindWavingHuman();
        success = response.success;

        if (success) {
            result_msg = "found: waving_human";
            
            // Guardamos el estado en el Knowledge
            knowledge->saveInRAM("last_event_detected", "waving_hand");
            
            RCLCPP_INFO(node->get_logger(), "FindWaving SUCCESS: Person located by waving gesture.");
        } else {
            result_msg = "not_found: scan_complete_no_gesture";
            RCLCPP_WARN(node->get_logger(), "FindWaving FAILED: No waving detected.");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "FindWaving CRASH: %s", e.what());
        result_msg = "not_found: hri_service_error";
    }

    // 3. Autorización final
    return executor.authorize(
        "find_waving",
        argument,
        result_msg,
        success ? "Persona saludando encontrada" : "No se detectó a nadie saludando"
    );
}