#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "hardware_tools/Hardware.hpp"

#include <array>
#include <memory>
#include <string>

// =========================================================
// Estados de la FSM para TestArms
// =========================================================
enum class TestArmsState {
    READY,
    RAISING_LEFT_ARM,
    WAVING_LEFT_ARM,
    LOWERING_LEFT_ARM,
    RAISING_RIGHT_ARM,
    WAVING_RIGHT_ARM,
    LOWERING_RIGHT_ARM,
    SUCCESS,
    FAILED
};

class TestArms
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    TestArms(const rclcpp::Node::SharedPtr&   node,
             const std::shared_ptr<Hardware>& hardware);

    Result run(const std::string&                 argument,
               const std::shared_ptr<GoalHandle>& gh);

private:
    // =========================================================
    // moveArm - Mueve el brazo indicado (6 DOF) a la pose dada
    // =========================================================
    bool moveArm(bool is_left, const std::array<double, 6>& pose);

    // =========================================================
    // waveArm - "Saluda": oscila de lado a lado, articulación
    //           por articulación (hombro lateral, codo sobre el
    //           eje del brazo y muñeca sobre el eje del brazo)
    // =========================================================
    bool waveArm(const std::shared_ptr<GoalHandle>& gh, bool is_left);

    // =========================================================
    // publishFeedback — Formato anidado fb->feedback.message
    // =========================================================
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         TestArmsState                      state,
                         const std::string&                 detail = "");

    // =========================================================
    // stateName - Converts state number to string
    // =========================================================
    static std::string stateName(TestArmsState s);

    // ---------------------------------------------------------
    // Articulaciones de cada brazo (índice en el arreglo de 6):
    //   0: hombro hacia adelante     1: hombro hacia los lados
    //   2: codo sobre el eje del brazo   3: codo hacia adelante
    //   4: muñeca sobre el eje del brazo 5: muñeca hacia adelante
    // ---------------------------------------------------------

    // Brazo en reposo (igual que ARM_RESTING en Give)
    static constexpr std::array<double, 6> LEFT_ARM_RESTING  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static constexpr std::array<double, 6> RIGHT_ARM_RESTING = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Pose de saludo: brazo levantado y antebrazo hacia arriba (PROPUESTA, refinar por brazo)
    static constexpr std::array<double, 6> LEFT_ARM_RAISED   = {1.2, 0.0, 0.0, 1.5, 0.0, 0.0};
    static constexpr std::array<double, 6> RIGHT_ARM_RAISED  = {1.2, 0.0, 0.0, 1.5, 0.0, 0.0};

    // Articulaciones que "saludan" (una tras otra) y su amplitud en rad (PROPUESTA)
    static constexpr std::array<int, 3>    WAVE_JOINTS     = {1, 2, 4};
    static constexpr std::array<double, 3> WAVE_AMPLITUDES = {0.30, 0.50, 0.50};
    static constexpr int                   WAVE_CYCLES     = 2;  // vaivenes por articulación

    rclcpp::Node::SharedPtr   node_;
    std::shared_ptr<Hardware> hardware_;
};
