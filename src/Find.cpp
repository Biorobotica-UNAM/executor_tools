#include "executor_tools/Find.hpp"
// =========================================================
// Constructor
// =========================================================
Find::Find(const rclcpp::Node::SharedPtr& node,
           const std::shared_ptr<Tasks>& tasks,
           const std::shared_ptr<Knowledge>& knowledge)
: node(node),
  tasks(tasks),
  knowledge(knowledge),
  executor(node)
{
}
// =========================================================
// Parsea "category_item" → {"category", "item"}
// =========================================================
std::pair<std::string, std::string> Find::parseCategoryItem(const std::string& category_item)
{
    size_t pos = category_item.find('_');
    if (pos == std::string::npos)
    {
        RCLCPP_WARN(node->get_logger(),
                    "No underscore found in '%s', using full string as category and empty item",
                    category_item.c_str());
        return {category_item, ""};
    }
    std::string category = category_item.substr(0, pos);
    if (pos + 1 >= category_item.size())
    {
        RCLCPP_WARN(node->get_logger(),
                    "No item found after '_' in '%s', item will be empty",
                    category_item.c_str());
        return {category, ""};
    }
    return {category, category_item.substr(pos + 1)};
}
// =========================================================
// Lee item desde Knowledge RAM
// =========================================================
std::string Find::resolveItem(const std::string& item)
{
    std::string value = knowledge->getNameFromLabel(item);
    
    // Si el valor devuelto es igual al item, significa que NO está en el YAML
    if (value == item)
    {
        RCLCPP_INFO(node->get_logger(),
                    "Find: '%s' no es un Label, se usará como nombre directo para la visión.",
                    item.c_str());
        return item; // <--- DEVOLVEMOS EL ORIGINAL en lugar de "./."
    }
    
    return value; // Devolvemos la traducción (ej. "cutlery" -> "fork")
}
// =========================================================
// Ejecuta find
// =========================================================
bool Find::run(const std::string& category_item,
               const std::shared_ptr<GoalHandle>& goalHandle)
{
    // 1. Verificación inicial de seguridad
    if (!goalHandle || !goalHandle->is_active()) return false;

    auto [category, item] = parseCategoryItem(category_item);

    // Si la categoría es waving o seat, no necesitamos un item.
    // Forzamos un item genérico para que no truene el resolveItem.
    if (category == "waving" && item.empty()) {
        item = "human"; 
    }
    
    // 2. Feedback con try-catch local
    try {
        auto feedback = std::make_shared<ExecuteTask::Feedback>();
        feedback->feedback.message = "Buscando: " + item + " (categoría: " + category + ")";
        if (goalHandle->is_active()) goalHandle->publish_feedback(feedback);
    } catch (...) { 
        RCLCPP_WARN(node->get_logger(), "Find: Could not publish feedback");
    }

    std::string resolved_item = resolveItem(item);

    // Si el item no se resolvió (es "./."), mejor ni le preguntamos a la visión
    if (resolved_item == "./." && category != "waving" && category != "seat") {
        return executor.authorize("find", category_item, "not_found: unknown_item", "Etiqueta no existe");
    }

    bool success = false;
    std::string result = "not_found: internal_error";

    // 3. Bloque de ejecución con protección de datos
    try {
        if (category == "object") {
            auto response = tasks->FindObject(resolved_item);
            success = response.success;
            
            if (success) {
                // 1. Usamos el nombre real que detectó la visión (ej. "tomato")
                std::string final_name = response.detected_class;

                // 2. Guardamos el nombre en las etiquetas de RAM
                knowledge->saveInRAM("last_object_found", final_name);
                knowledge->updateLabel("object_1", final_name);
                knowledge->updateLabel("last_object_found", final_name);

                // 3. CORRECCIÓN: Guardamos la pose bajo el nombre real Y bajo la etiqueta[cite: 8]
                // Esto asegura que si alguien busca "last_object_found", obtenga estas coordenadas.
                knowledge->updateObjectPose(final_name, response.point);
                knowledge->updateObjectPose("last_object_found", response.point);

                result = "found: " + final_name + 
                        " at x=" + std::to_string(response.point.point.x) + 
                        ", y=" + std::to_string(response.point.point.y);
                
                RCLCPP_INFO(node->get_logger(), "Find SUCCESS: %s", result.c_str());
            } else {
                // Manejo de error o no encontrado
                result = "not_found: " + (response.message.empty() ? "search_failed" : response.message);
                RCLCPP_WARN(node->get_logger(), "Find FAILED: %s", result.c_str());
            }
        }
        else if (category == "person") {
            std::string person_to_find = (resolved_item == "anybody") ? "" : resolved_item;
            auto response = tasks->FindPerson(person_to_find);
            success = response.success;
            result = success ? "found: " + response.name : "not_found: person not recognized";
        }
        else if (category == "seat") {
            auto response = tasks->FindSeat();
            success = response.success;
            result = success ? "found: seat available" : "not_found: " + response.message;
        }
        else if (category == "waving") {
            auto response = tasks->FindWavingHuman(); 
            success = response.success;
            
            if (success) {
                result = "found: waving_human";
            } else {
                result = "not_found: no_one_waving";
            }
        }
        else {
            RCLCPP_ERROR(node->get_logger(), "Find: unknown category '%s'", category.c_str());
            result = "not_found: unknown category";
            success = false;
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Find: Task call crashed: %s", e.what());
        success = false;
        result = "not_found: exception_in_vision_layer";
    }

    // 4. Authorize final
    return executor.authorize(
        "find",
        category_item,
        result,
        success ? "Objeto localizado" : "Fallo en localización"
    );
}
