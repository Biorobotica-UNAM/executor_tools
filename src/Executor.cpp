#include "executor_tools/Executor.hpp"

Executor::Executor(const rclcpp::Node::SharedPtr& node)
: node(node)
{
  client = node->create_client<AuthorizeTask>(
    "/supervisor/supervisor_executor/authorize"
  );
}

bool Executor::authorize(const std::string& action,
                         const std::string& argument,
                         const std::string& worker_result,
                         const std::string& context_message)
{
  // OJO: esto solo espera que exista el servicio; no hace spin
  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Supervisor authorization service not available.");
    return false;
  }

  auto req = std::make_shared<AuthorizeTask::Request>();
  req->action = action;
  req->argument = argument;
  req->worker_result = worker_result;
  req->context_message = context_message;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    waiting_ = true;
    last_approved_.reset();
    last_message_.clear();
  }

  // Enviar async y resolver por callback (sin spin interno)
  client->async_send_request(req,
    [this](rclcpp::Client<AuthorizeTask>::SharedFuture future)
    {
      bool approved = false;
      std::string message;

      try {
        auto res = future.get();
        approved = res->approved;
        message  = res->message;
      } catch (const std::exception &e) {
        approved = false;
        message  = std::string("exception: ") + e.what();
      }

      {
        std::lock_guard<std::mutex> lk(mtx_);
        last_approved_ = approved;
        last_message_ = message;
        waiting_ = false;
      }
      cv_.notify_all();
    });

  // Espera “bloqueante” pero SIN spin.
  // (El spin lo está haciendo tu main/executor, por eso el callback sí llega.)
  std::unique_lock<std::mutex> lk(mtx_);
  const bool ok = cv_.wait_for(lk, std::chrono::seconds(3), [this] {
    return !waiting_ && last_approved_.has_value();
  });

  if (!ok)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Authorization request timed out (no response).");
    return false;
  }

  // opcional: log del mensaje
  if (!last_message_.empty()) {
    RCLCPP_INFO(node->get_logger(), "Authorize response: %s", last_message_.c_str());
  }

  return *last_approved_;
}
