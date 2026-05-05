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

    // 1. Feedback inicial
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.message = "Buscando a la persona: " + name_to_find;
    goalHandle->publish_feedback(feedback);

    // 2. Resolución del nombre (Habilitamos búsqueda por etiquetas)
    std::string final_name = resolvePersonName(name_to_find);

    RCLCPP_INFO(node->get_logger(), "FindPerson: Starting search for '%s'", 
                final_name.empty() ? "anybody" : final_name.c_str());

    bool success = false;
    std::string result_msg = "not_found: person_not_recognized";

    try {
        // 3. Llamada al servicio de reconocimiento facial
        auto response = tasks->FindPerson(final_name);
        success = response.success;

        if (success) {
            result_msg = "found: " + response.name;
            
            // Actualizamos el Knowledge con la persona encontrada
            knowledge->saveInRAM("last_person_found", response.name);
            knowledge->updateLabel("person_1", response.name);
            
            RCLCPP_INFO(node->get_logger(), "FindPerson SUCCESS: Found %s", response.name.c_str());
        } else {
            RCLCPP_WARN(node->get_logger(), "FindPerson FAILED: Person not in sight");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "FindPerson CRASH: %s", e.what());
        result_msg = "not_found: internal_error";
    }

    // 4. Autorización final al Supervisor
    return executor.authorize(
        "find_person",
        name_to_find,
        result_msg,
        success ? "Persona localizada" : "No se encontró a la persona"
    );
}