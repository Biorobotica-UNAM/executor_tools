#include "executor_tools/Unfocus.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Unfocus::Unfocus(const rclcpp::Node::SharedPtr& node)
: node(node),
  executor(node)
{
}

// =========================================================
// Ejecuta unfocus (llama a /vision/focus con data = false)
// =========================================================
bool Unfocus::run(const std::string& args,
                  const std::shared_ptr<GoalHandle>& goalHandle)
{
    // -------------------- FEEDBACK --------------------
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.step_id = 1;
    feedback->feedback.progress = 0.5f;
    feedback->feedback.message  = "Desactivando focus de visión";
    goalHandle->publish_feedback(feedback);

    // -------------------- SERVICE CALL --------------------
    auto aux_node = rclcpp::Node::make_shared("unfocus_aux_node");
    auto client   = aux_node->create_client<std_srvs::srv::SetBool>("/vision/focus");

    rclcpp::executors::SingleThreadedExecutor aux_exec;
    aux_exec.add_node(aux_node);

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Unfocus: rclcpp interrupted while waiting for service");
            return executor.authorize("unfocus", args, "error: interrupted", "Unfocus fallido");
        }
        RCLCPP_WARN(node->get_logger(), "Unfocus: waiting for /vision/focus...");
    }

    auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;

    auto future = client->async_send_request(request);

    auto status = aux_exec.spin_until_future_complete(future, 5s);

    bool        success = false;
    std::string result;

    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
        success = future.get()->success;
        result  = success
            ? "focus: deactivated"
            : "unfocus: rejected — " + future.get()->message;
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Unfocus: service call timed out or interrupted");
        result = "error: service call timed out";
    }

    // -------------------- AUTHORIZE --------------------
    return executor.authorize(
        "unfocus",
        args,
        result,
        success ? "Focus desactivado" : "Unfocus fallido"
    );
}
