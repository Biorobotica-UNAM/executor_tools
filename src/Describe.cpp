#include "executor_tools/Describe.hpp"
#include <future>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

// =========================================================
// Constructor
// =========================================================
Describe::Describe(const rclcpp::Node::SharedPtr &node,
                   const std::shared_ptr<Knowledge> &knowledge)
    : node(node), knowledge(knowledge), executor(node) {
    client = node->create_client<faces_interfaces::srv::FaceRecog>("/vision/describe_person");
}

// =========================================================
// Ejecuta Describe
// =========================================================
bool Describe::run(const std::string &person_name,
                   const std::shared_ptr<GoalHandle> &goalHandle) {

    // -------------------- FEEDBACK --------------------
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.message = "Describing: " + person_name;
    goalHandle->publish_feedback(feedback);

    // -------------------- SERVICE CHECK --------------------
    if (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node->get_logger(), "Vision service not found");
        return false;
    }

    // -------------------- ASYNC REQUEST (PROMISE) --------------------
    auto request = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
    request->name = person_name;

    auto promise = std::make_shared<std::promise<faces_interfaces::srv::FaceRecog::Response::SharedPtr>>();
    auto future = promise->get_future();

    auto callback = [promise](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture response_future) {
        promise->set_value(response_future.get());
    };

    client->async_send_request(request, callback);

    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        RCLCPP_ERROR(node->get_logger(), "Vision service timed out");
        return false;
    }

    auto response = future.get();
    if (!response->success) return false;

    std::string description = response->description;

    // -------------------- KNOWLEDGE SUFFIX LOGIC --------------------
    // Determinamos si los datos van a guest_1 o guest_2
    std::string suffix = "";
    if (person_name == knowledge->getNameFromLabel("name_1")) suffix = "_1";
    else if (person_name == knowledge->getNameFromLabel("name_2")) suffix = "_2";
    else return executor.authorize("Describe", person_name, "failure", "Name not in interest_words");

    // -------------------- STRING PARSER (ATTRIBUTES) --------------------
    // Género
    std::vector<std::string> genders = {"female", "woman", "male", "men", "boy", "person"};
    for (const auto& g : genders) {
        if (description.find(g) != std::string::npos) {
            knowledge->updateLabel("guest_gender" + suffix, g);
            break;
        }
    }
    // Edad
    std::vector<std::string> ages = {"adult", "young", "elderly"};
    for (const auto& a : ages) {
        if (description.find(a) != std::string::npos) {
            knowledge->updateLabel("guest_age" + suffix, a);
            break;
        }
    }
    // Colores de ropa
    std::vector<std::string> colors = {"green", "red", "black", "white", "blue", "yellow", "gray", "grey", "navy", "orange", "purple"};
    for (const auto& c : colors) {
        if (description.find(c) != std::string::npos) {
            knowledge->updateLabel("guest_shirt_color" + suffix, c);
            break;
        }
    }

    knowledge->saveToFileDirect();
    return executor.authorize("Describe", person_name, "success", "Info saved for " + person_name);
}