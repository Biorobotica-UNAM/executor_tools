#include "executor_tools/FindObject.hpp"

FindObject::FindObject(const rclcpp::Node::SharedPtr& node,
                       const std::shared_ptr<Tasks>& tasks,
                       const std::shared_ptr<Knowledge>& knowledge)
: node(node), tasks(tasks), knowledge(knowledge), executor(node)
{}

std::string FindObject::resolveObjectName(const std::string& label)
{
    std::string resolved = knowledge->getNameFromLabel(label);
    // Si no es una etiqueta conocida, devolvemos el original para la visión directa
    return (resolved == label || resolved == "./.") ? label : resolved;
}

bool FindObject::run(const std::string& object_label,
                     const std::shared_ptr<GoalHandle>& goalHandle)
{
    if (!goalHandle || !goalHandle->is_active()) return false;

    // 1. Feedback al Supervisor
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.message = "Iniciando escaneo para: " + object_label;
    goalHandle->publish_feedback(feedback);

    // 2. Traducción de etiqueta (ej. "beverage" -> "coke")
    std::string target_name = resolveObjectName(object_label);
    RCLCPP_INFO(node->get_logger(), "FindObject: Looking for '%s'", target_name.c_str());

    bool success = false;
    std::string result_msg = "not_found: object_not_seen";

    try {
        // 3. Llamada a tu nueva FSM (la que probaste con service call)
        auto response = tasks->FindObject(target_name);
        success = response.success;

        if (success) {
            std::string final_class = response.detected_class;

            // 4. GUARDADO ESTRATÉGICO EN KNOWLEDGE
            // Guardamos el nombre real detectado
            knowledge->saveInRAM("last_object_found", final_class);
            knowledge->updateLabel("object_1", final_class);
            
            // Guardamos la Pose bajo el nombre real Y bajo la etiqueta genérica
            // Esto permite que el brazo busque "last_object_found" sin saber qué es.
            knowledge->updateObjectPose(final_class, response.point);
            knowledge->updateObjectPose("last_object_found", response.point);

            result_msg = "found: " + final_class + 
                         " at x=" + std::to_string(response.point.point.x)+ 
                         " at y=" + std::to_string(response.point.point.y)+ 
                         " at z=" + std::to_string(response.point.point.z);
            
            RCLCPP_INFO(node->get_logger(), "FindObject SUCCESS: %s", final_class.c_str());
        } else {
            result_msg = "not_found: " + (response.message.empty() ? "scan_finished_without_success" : response.message);
            RCLCPP_WARN(node->get_logger(), "FindObject FAILED: %s", result_msg.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "FindObject CRASH: %s", e.what());
        result_msg = "not_found: vision_service_error";
    }

    // 5. Autorización final
    return executor.authorize(
        "find_object",
        object_label,
        result_msg,
        success ? "Objeto encontrado y posicionado" : "No se pudo localizar el objeto"
    );
}