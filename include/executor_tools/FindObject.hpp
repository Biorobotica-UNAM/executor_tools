#pragma once

#include <memory>
#include <string>
#include <vector>
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hardware_tools/Hardware.hpp"
#include "vision_tools/Vision.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp" 

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"

#include <string>
#include <vector>
#include <memory> 

enum class FindObjectState {
    READY,
    MOVING_HEAD,
    STABILIZING,
    DETECTING,
    LOOKING_AT_TARGET,
    SUCCESS,
    FAILED
};

class FindObject
{
public:

    using ExecuteTask = executor_interfaces::action::ExecuteTask;

    using GoalHandle =
        rclcpp_action::ServerGoalHandle<ExecuteTask>;

    enum class FindObjectState
    {
        READY,
        MOVING_HEAD,
        STABILIZING,
        DETECTING,
        LOOKING_AT_TARGET,
        SUCCESS,
        FAILED
    };

    FindObject(
        const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<Hardware>& hardware,
        const std::shared_ptr<Vision>& vision);

    Result run(
        const std::string& object_name,
        const std::shared_ptr<GoalHandle>& gh);

private:

    bool detectWithOWL(
        const std::string& object_name,
        geometry_msgs::msg::PoseStamped& pose);

    bool detectEmptyChair(
        geometry_msgs::msg::PointStamped& point);

    void publishFeedback(
        const std::shared_ptr<GoalHandle>& gh,
        FindObjectState state,
        const std::string& detail = "");

    std::string stateName(FindObjectState s);

private:

    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<Hardware> hardware_;

    std::shared_ptr<Vision> vision_;

    // 3. AGREGA ESTO: Declarar los objetos de TF2 que inicializas en el constructor del .cpp
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 4. AGREGA ESTO: Las constantes numéricas que usa tu función detectWithOWL
    static constexpr int    MAX_OWL_INSTANCES = 5;       // Ajusta el número si es necesario
    static constexpr double MAX_TF_AGE_S      = 2.0;     // Segundos máximos de antigüedad
    static constexpr double MIN_Z_THRESHOLD   = 0.2;     // Umbral de altura mínima
    static constexpr double MIN_Z_HEIGHT      = 0.5;     // Altura de seguridad corregida

    const std::vector<std::pair<double,double>> scan_poses_ = {
        { 0.0,  -0.35},
        {-0.35, -0.45},
        { 0.35, -0.45},
        { 0.0,  -0.60},
        {-0.25, -0.65},
        { 0.25, -0.65}
    };
};