#include "executor_tools/Follow.hpp"
#include <chrono>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Follow::Follow(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
    // Cliente apuntando al nodo de tracking humano de Justina
    client_ = node_->create_client<std_srvs::srv::SetBool>("/hri/human_tracking/enable");
}

// =========================================================
// run — Filtra argumentos y ejecuta el llamado al servicio
// =========================================================
Result Follow::run(const std::string& argument,
                   const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    // Valida que el argumento sea correcto
    if (argument != "enable" && argument != "disable") {
        RCLCPP_WARN(node_->get_logger(), "Follow: Unknown argument '%s'", argument.c_str());
        return {false, "FAILED", "unknown_argument: " + argument};
    }

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, FollowState::READY, argument);

    // Espera del servicio por máximo 2 segundos
    if (!client_->wait_for_service(2s)) {
        RCLCPP_ERROR(node_->get_logger(), "Follow: Tracking service unavailable.");
        publishFeedback(gh, FollowState::FAILED, "service_unavailable");
        return {false, "FAILED", "tracking_service_unavailable"};
    }

    // ── PENDING_SERVICE ───────────────────────────────────
    publishFeedback(gh, FollowState::PENDING_SERVICE, "Sending request...");

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    // Si es "enable" manda true (enciende), si es "disable" manda false (apaga)
    request->data = (argument == "enable");

    auto result_future = client_->async_send_request(request);

    bool service_success = false;
    // Espera máxima de 1.5 segundos por la respuesta del nodo HRI
    if (result_future.wait_for(1500ms) == std::future_status::ready) {
        auto response = result_future.get();
        service_success = response->success;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Follow: Service response timed out.");
    }

    // ── SUCCESS / FAILED ──────────────────────────────────
    if (service_success) {
        std::string status_msg = (argument == "enable") ? "tracking_activated" : "tracking_deactivated";
        publishFeedback(gh, FollowState::SUCCESS, status_msg);
        return {true, "SUCCESS", status_msg};
    } else {
        publishFeedback(gh, FollowState::FAILED, "service_rejected_request");
        return {false, "FAILED", "tracking_toggle_failed"};
    }
}

// =========================================================
// publishFeedback — Formato anidado fb->feedback.message
// =========================================================
void Follow::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                            FollowState                        state,
                            const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[follow] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg; // TOTALMENTE ASOCIADO A TU .ACTION ANIDADO REAL
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string Follow::stateName(FollowState s)
{
    switch (s) {
        case FollowState::READY:           return "READY";
        case FollowState::PENDING_SERVICE: return "PENDING_SERVICE";
        case FollowState::SUCCESS:         return "SUCCESS";
        case FollowState::FAILED:          return "FAILED";
        default:                           return "UNKNOWN";
    }
}