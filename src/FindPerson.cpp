#include "executor_tools/FindPerson.hpp"

FindPerson::FindPerson(const rclcpp::Node::SharedPtr& node,
                       const std::shared_ptr<Tasks>& tasks,
                       const std::shared_ptr<Knowledge>& knowledge)
: node(node), tasks(tasks), knowledge(knowledge), executor(node)
{}

std::string FindPerson::resolvePersonName(const std::string& name)
{
    // Intentamos resolver si 'name' es una etiqueta (ej. "operator")
    std::string resolved = knowledge->getNameFromLabel(name);
    
    // Si el Knowledge devuelve lo mismo, es un nombre directo o "anybody"
    if (resolved == name || resolved == "./.") {
        return (name == "anybody") ? "" : name;
    }
    return resolved;
}

bool FindPerson::run(const std::string& name_to_find,
                     const std::shared_ptr<GoalHandle>& goalHandle)
{
    if (!goalHandle || !goalHandle->is_active()) return false;

    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.message = "Buscando a: " + name_to_find;
    goalHandle->publish_feedback(feedback);

    std::string final_name = resolvePersonName(name_to_find);
    bool success = false;
    std::string result_msg = "not_found: person_not_recognized";

    try {
        // 3. Llamada al servicio (El nodo FSM hará el paneo/tilteo)
        auto response = tasks->FindPerson(final_name);
        success = response.success;

        if (success) {
            // El nombre que realmente devolvió el nodo (puede ser el original o uno nuevo)
            std::string actual_name = response.name; 
            result_msg = "found: " + actual_name;
            
            // --- ACTUALIZACIÓN DE KNOWLEDGE ---
            knowledge->saveInRAM("last_person_found", actual_name);
            
            // Si el Supervisor pidió buscar a "guest", actualizamos el label específico
            if (name_to_find.find("guest") != std::string::npos || name_to_find.find("name_") != std::string::npos) {
                 knowledge->updateLabel(name_to_find, actual_name);
            }

            // OPCIONAL: Si el .srv trae la descripción de Florence-2, guárdala de una vez
            if (!response.description.empty()) {
                knowledge->saveInRAM("last_description", response.description);
                RCLCPP_INFO(node->get_logger(), "Description received: %s", response.description.c_str());
            }
            
            RCLCPP_INFO(node->get_logger(), "FindPerson SUCCESS: Found %s", actual_name.c_str());
        } 
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "FindPerson CRASH: %s", e.what());
        result_msg = "not_found: internal_error";
    }

    return executor.authorize("find_person", name_to_find, result_msg, 
                            success ? "Persona localizada" : "No se encontró");
}