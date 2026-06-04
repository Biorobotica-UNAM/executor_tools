#pragma once

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Interfaces y herramientas del Executor
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"
#include "vision_tools/Vision.hpp"
#include "navigation_tools/Navigation.hpp"
#include "knowledge_tools/Knowledge.hpp"

// Componentes de TF2 de ROS 2
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// ─────────────────────────────────────────────────────────────
//  Estados de la FSM de Aproximación
// ─────────────────────────────────────────────────────────────
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

    // Constructor con inyección de dependencias (Coincide con tu initTools)
    Approach(const rclcpp::Node::SharedPtr&        node,
             const std::shared_ptr<Hardware>&      hardware,
             const std::shared_ptr<Vision>&        vision,
             const std::shared_ptr<Navigation>&    navigation,
             const std::shared_ptr<Knowledge>&     knowledge);

    ~Approach() = default;

    // Punto de entrada principal disparado por el Worker
    Result run(const std::string&                 argument,
               const std::shared_ptr<GoalHandle>& gh);

private:
    // Publicador de feedback hacia el Action Server
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         ApproachState                      state,
                         const std::string&                 detail = "");

    // Convertidor de enum a string para logs y tópicos
    std::string stateName(ApproachState s);

    // Miembros del nodo y herramientas compartidas
    rclcpp::Node::SharedPtr        node_;
    std::shared_ptr<Hardware>      hardware_;
    std::shared_ptr<Vision>        vision_;
    std::shared_ptr<Navigation>    navigation_;
    std::shared_ptr<Knowledge>     knowledge_;

    // Tuberías de transformaciones (Mantenidas por compatibilidad de la clase)
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};