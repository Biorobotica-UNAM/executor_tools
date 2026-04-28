#include "executor_tools/Focus.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Focus::Focus(const rclcpp::Node::SharedPtr& node)
: node(node),
  executor(node)
{
}

// =========================================================
// Ejecuta focus (llama a /vision/focus con data = true)
// =========================================================
bool Focus::run(const std::string& args,
                const std::shared_ptr<GoalHandle>& goalHandle)
{
    // -------------------- FEEDBACK --------------------
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.step_id = 1;
    feedback->feedback.progress = 0.5f;
    feedback->feedback.message  = "Activando focus de visión";
    goalHandle->publish_feedback(feedback);

    // -------------------- SERVICE CALL --------------------
    // Nodo auxiliar con su propio executor, completamente independiente
    // del nodo principal que ya está siendo spinneado en main()
    auto aux_node = rclcpp::Node::make_shared("focus_aux_node");
    auto client   = aux_node->create_client<std_srvs::srv::SetBool>("/vision/focus");

    rclcpp::executors::SingleThreadedExecutor aux_exec;
    aux_exec.add_node(aux_node);

    // Esperar servicio
    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Focus: rclcpp interrupted while waiting for service");
            return executor.authorize("focus", args, "error: interrupted", "Focus fallido");
        }
        RCLCPP_WARN(node->get_logger(), "Focus: waiting for /vision/focus...");
    }

    auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto future = client->async_send_request(request);

    // Spin el aux_executor hasta que el future esté listo
    auto status = aux_exec.spin_until_future_complete(future, 5s);

    bool        success = false;
    std::string result;

    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
        success = future.get()->success;
        result  = success
            ? "focus: activated"
            : "focus: rejected — " + future.get()->message;
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Focus: service call timed out or interrupted");
        result = "error: service call timed out";
    }

    // -------------------- AUTHORIZE --------------------
    return executor.authorize(
        "focus",
        args,
        result,
        success ? "Focus activado" : "Focus fallido"
    );
}
