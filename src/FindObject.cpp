#include "executor_tools/FindObject.hpp"

FindObject::FindObject(const rclcpp::Node::SharedPtr& node,
                       const std::shared_ptr<Tasks>& tasks,
                       const std::shared_ptr<Knowledge>& knowledge)
: node(node), tasks(tasks), knowledge(knowledge), executor(node)
{}

std::string FindObject::resolveObjectName(const std::string& label)
{
    // Intentamos traducir si es una etiqueta (ej. "drink" -> "coke")
    std::string resolved = knowledge->getNameFromLabel(label);
    return (resolved == label || resolved == "./.") ? label : resolved;
}

bool FindObject::run(const std::string& object_label,
                     const std::shared_ptr<GoalHandle>& goalHandle)
{
    if (!goalHandle || !goalHandle->is_active()) return false;

    // 1. Feedback inicial para el Supervisor
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.message = "Escaneando y calculando TF para: " + object_label;
    goalHandle->publish_feedback(feedback);

    std::string target_name = resolveObjectName(object_label);
    RCLCPP_INFO(node->get_logger(), "FindObject: Looking for '%s'", target_name.c_str());

    bool success = false;
    std::string result_msg = "not_found";

    try {
        // 2. Llamada a la FSM mediante el wrapper de Tasks
        // Ahora esperamos un PoseStamped en la respuesta
        auto response = tasks->FindObject(target_name);
        success = response.success;

        if (success) {
            std::string final_class = response.detected_class;

            // Extraemos datos de Pose (Position + Orientation)
            const auto &p = response.pose.pose.position;
            const auto &o = response.pose.pose.orientation;

            // 3. GUARDADO EN RAM (Knowledge Bank)
            // Guardamos el nombre real detectado
            knowledge->saveInRAM("last_object_found", final_class);
            knowledge->updateLabel("object_1", final_class);
            
            // Guardamos la Pose completa (PoseStamped) para que el brazo tenga orientación
            knowledge->updateObjectPose(final_class, response.pose);
            knowledge->updateObjectPose("last_object_found", response.pose);

            // 4. CONSTRUCCIÓN DEL MENSAJE DETALLADO
            // Usamos substr para no saturar la terminal con decimales de flotantes
            result_msg = "found:" + final_class + 
                         " pos[x:" + std::to_string(p.x) + 
                         ",y:" + std::to_string(p.y) + 
                         ",z:" + std::to_string(p.z) + "]" +
                         " ori[qx:" + std::to_string(o.x) + 
                         ",qy:" + std::to_string(o.y) + 
                         ",qz:" + std::to_string(o.z) + 
                         ",qw:" + std::to_string(o.w)+ "]";
            
            RCLCPP_INFO(node->get_logger(), "FindObject SUCCESS: %s", result_msg.c_str());
        } else {
            result_msg = "not_found";
            RCLCPP_WARN(node->get_logger(), "FindObject: Object '%s' not found during scan.", target_name.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "FindObject CRASH: %s", e.what());
        result_msg = "not_found";
    }

    // 5. Autorización final al Supervisor con el reporte de ejecución
    return executor.authorize(
        "find_object",
        object_label,
        result_msg,
        success ? "Objeto localizado y orientado en base_link" : "No se pudo localizar el objeto"
    );
}