#pragma once

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"
#include "vision_tools/Vision.hpp"
#include "navigation_tools/Navigation.hpp"
#include "knowledge_tools/Knowledge.hpp"

#include "known_locations_interfaces/srv/save_pose_location.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Estados de la FSM de Aproximación
enum class ApproachState {
    READY,
    LOADING,
    APPROACHING,
    SUCCESS,
    FAILED
};

class Approach {
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Approach(const rclcpp::Node::SharedPtr&        node,
             const std::shared_ptr<Hardware>&      hardware,
             const std::shared_ptr<Vision>&        vision,
             const std::shared_ptr<Navigation>&    navigation,
             const std::shared_ptr<Knowledge>&     knowledge);

    ~Approach() = default;

    Result run(const std::string&                 argument,
               const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         ApproachState                      state,
                         const std::string&                 detail = "");

    std::string stateName(ApproachState s);

    // Miembros del nodo y herramientas compartidas
    rclcpp::Node::SharedPtr        node_;
    std::shared_ptr<Hardware>      hardware_;
    std::shared_ptr<Vision>        vision_;
    std::shared_ptr<Navigation>    navigation_;
    std::shared_ptr<Knowledge>     knowledge_;

    rclcpp::Client<known_locations_interfaces::srv::SavePoseLocation>::SharedPtr save_loc_client_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};