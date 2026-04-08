#include "executor_tools/Describe.hpp"
#include <future>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

Describe::Describe(const rclcpp::Node::SharedPtr &node,
                   const std::shared_ptr<Knowledge> &knowledge)
    : node(node), knowledge(knowledge), executor(node) {
    client = node->create_client<faces_interfaces::srv::FaceRecog>("/vision/describe_person");
}

bool Describe::run(const std::string &person_name,
                   const std::shared_ptr<GoalHandle> &goalHandle) {

    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.message = "Describing: " + person_name;
    goalHandle->publish_feedback(feedback);

    if (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node->get_logger(), "Vision service not found");
        return false;
    }

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

    // --- LÓGICA DE ACTUALIZACIÓN DE KNOWLEDGE ---
    std::string suffix = "";
    if (person_name == knowledge->getNameFromLabel("name_1")) suffix = "_1";
    else if (person_name == knowledge->getNameFromLabel("name_2")) suffix = "_2";
    else return executor.authorize("Describe", person_name, "failure", "Name not in interest_words");

    // Parser simplificado ( male, young, glasses... )
    if (description.find("male") != std::string::npos) knowledge->updateLabel("guest_gender" + suffix, "man");
    else if (description.find("female") != std::string::npos) knowledge->updateLabel("guest_gender" + suffix, "woman");

    if (description.find("young") != std::string::npos) knowledge->updateLabel("guest_age" + suffix, "young");
    else if (description.find("adult") != std::string::npos) knowledge->updateLabel("guest_age" + suffix, "adult");

    // Colores de ropa
    std::vector<std::string> colors = {"grey", "blue", "red", "black", "white"};
    for (const auto& c : colors) {
        if (description.find(c) != std::string::npos) {
            knowledge->updateLabel("guest_shirt_color" + suffix, c);
            break;
        }
    }

    knowledge->saveToFileDirect();
    return executor.authorize("Describe", person_name, "success", "Info saved for " + person_name);
}