#ifndef EXECUTOR_TOOLS__APPROACH_HPP_
#define EXECUTOR_TOOLS__APPROACH_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "executor_tools/Result.hpp" 

class Hardware;
class Vision;
class Navigation;

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
    using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Approach(const rclcpp::Node::SharedPtr &node,
             const std::shared_ptr<Hardware> &hardware,
             const std::shared_ptr<Vision> &vision,
             const std::shared_ptr<Knowledge> &knowledge,
             const std::shared_ptr<Navigation> &navigation);

    ~Approach() = default;

    Result run(const std::string &argument,
               const std::shared_ptr<GoalHandle> &gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle> &gh, 
                         ApproachState state, 
                         const std::string &detail = "");
    
    std::string stateName(ApproachState s);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Hardware> hardware_;
    std::shared_ptr<Vision> vision_;
    std::shared_ptr<Knowledge> knowledge_;
    std::shared_ptr<Navigation> navigation_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif