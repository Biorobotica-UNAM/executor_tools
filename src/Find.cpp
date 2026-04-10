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
    if (value == item)
    {
        RCLCPP_WARN(node->get_logger(),
                    "Find: Label '%s' not found in Knowledge RAM, returning empty",
                    item.c_str());
        return "./.";
    }
    return value;
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
    if (resolved_item == "./.") {
        RCLCPP_ERROR(node->get_logger(), "Find: Item '%s' not found in Knowledge. Skipping vision call.", item.c_str());
        return executor.authorize("find", category_item, "not_found: unknown_item", "Etiqueta no existe en memoria");
    }
    
    bool success = false;
    std::string result = "not_found: internal_error";

    // 3. Bloque de ejecución con protección de datos
    try {
        if (category == "object") {
            auto response = tasks->FindObject(resolved_item);
            success = response.success;
            
            if (success) {
                // SOLO accedemos a las coordenadas si success es true
                result = "found: " + response.detected_class +
                         " x=" + std::to_string(response.point.point.x);
            } else {
                result = "not_found: " + (response.message.empty() ? "timeout/no_obj" : response.message);
            }
        } 
        else if (category == "person") {
            auto response = tasks->FindPerson(resolved_item);
            success = response.success;
            result = success ? "found: " + response.name : "not_found: person not recognized";
        }
        else if (category == "seat") {
            auto response = tasks->FindSeat();
            success = response.success;
            result = success ? "found: seat available" : "not_found: " + response.message;
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
