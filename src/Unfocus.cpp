#include "executor_tools/Unfocus.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>
#include <mutex>
#include <condition_variable>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Unfocus::Unfocus(const rclcpp::Node::SharedPtr& node)
: node(node),
  executor(node)
{
    client = node->create_client<std_srvs::srv::SetBool>("/vision/focus");
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
    if (!client->wait_for_service(3s))
    {
        RCLCPP_ERROR(node->get_logger(), "Unfocus: service /vision/focus not available");
        return executor.authorize("unfocus", args, "error: service unavailable", "Unfocus fallido");
    }

    auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;

    std::mutex mtx;
    std::condition_variable cv;
    bool done    = false;
    bool success = false;
    std::string result;

    auto callback = [&](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
    {
        auto response = future.get();
        success = response->success;
        result  = success ? "focus: deactivated" : "unfocus: rejected — " + response->message;
        done    = true;
        cv.notify_one();
    };

    client->async_send_request(request, callback);

    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 5s, [&]{ return done; }))
    {
        RCLCPP_ERROR(node->get_logger(), "Unfocus: service call timed out");
        result  = "error: service call timed out";
        success = false;
    }

    // -------------------- AUTHORIZE --------------------
    return executor.authorize(
        "unfocus",
        args,
        result,
        success ? "Focus desactivado" : "Unfocus fallido"
    );
}
