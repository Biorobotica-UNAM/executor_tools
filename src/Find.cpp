#include "executor_tools/Find.hpp"

// =========================================================
// Constructor
// =========================================================
Find::Find(const rclcpp::Node::SharedPtr& node,
           const std::shared_ptr<Tasks>& tasks)
: node(node),
  tasks(tasks),
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
                    "No underscore found in '%s', using full string as item",
                    category_item.c_str());
        return {"", category_item};
    }
    return {category_item.substr(0, pos), category_item.substr(pos + 1)};
}

// =========================================================
// Ejecuta find
// =========================================================
bool Find::run(const std::string& category_item,
               const std::shared_ptr<GoalHandle>& goalHandle)
{
    // -------------------- PARSE --------------------
    auto [category, item] = parseCategoryItem(category_item);

    // -------------------- FEEDBACK --------------------
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.step_id = 1;
    feedback->feedback.progress = 0.5f;
    feedback->feedback.message  = "Buscando: " + item + " (categoría: " + category + ")";
    goalHandle->publish_feedback(feedback);

    // -------------------- TASKS CALL --------------------
    auto response = tasks->Find(category, item);

    // -------------------- RESULT STRING --------------------
    std::string result = response.success
        ? "found: "     + response.detected_class +
          " conf="      + std::to_string(response.confidence) +
          " x="         + std::to_string(response.point.point.x) +
          " y="         + std::to_string(response.point.point.y) +
          " z="         + std::to_string(response.point.point.z)
        : "not_found: " + response.message;

    // -------------------- AUTHORIZE --------------------
    return executor.authorize(
        "find",
        category_item,
        result,
        response.success ? "Objeto localizado" : "Objeto no encontrado"
    );
}
