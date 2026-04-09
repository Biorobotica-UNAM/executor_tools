#include "executor_tools/Focus.hpp"
#include "std_srvs/srv/set_bool.hpp"

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
    auto client = node->create_client<std_srvs::srv::SetBool>("/vision/focus");

    if (!client->wait_for_service(std::chrono::seconds(3)))
    {
        RCLCPP_ERROR(node->get_logger(), "Focus: service /vision/focus not available");
        return executor.authorize("focus", args, "error: service unavailable", "Focus fallido");
    }

    auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto future = client->async_send_request(request);

    bool        success = false;
    std::string result;

    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5))
        == rclcpp::FutureReturnCode::SUCCESS)
    {
        success = future.get()->success;
        result  = success ? "focus: activated" : "focus: rejected — " + future.get()->message;
    }
    else
    {
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
