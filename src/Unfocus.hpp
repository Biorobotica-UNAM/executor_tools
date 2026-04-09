#include "executor_tools/Unfocus.hpp"
#include "std_srvs/srv/set_bool.hpp"

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
    auto client = node->create_client<std_srvs::srv::SetBool>("/vision/focus");

    if (!client->wait_for_service(std::chrono::seconds(3)))
    {
        RCLCPP_ERROR(node->get_logger(), "Unfocus: service /vision/focus not available");
        return executor.authorize("unfocus", args, "error: service unavailable", "Unfocus fallido");
    }

    auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;

    auto future = client->async_send_request(request);

    bool        success = false;
    std::string result;

    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5))
        == rclcpp::FutureReturnCode::SUCCESS)
    {
        success = future.get()->success;
        result  = success ? "focus: deactivated" : "unfocus: rejected — " + future.get()->message;
    }
    else
    {
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
