#ifndef EXECUTOR_TOOLS__APPROACH_HPP_
#define EXECUTOR_TOOLS__APPROACH_HPP_
#include <memory>
#include <mutex>
#include <string>
#include "actionlib_msgs/msg/goal_status.hpp"
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
    Result runPartial(const std::string &argument,
                      const std::shared_ptr<GoalHandle> &gh);
private:
    struct NavigationStatusSnapshot {
        bool has_status{false};
        bool done{false};
        bool success{false};
        int code{-1};
        std::string text;
    };
    void publishFeedback(const std::shared_ptr<GoalHandle> &gh, 
                         ApproachState state, 
                         const std::string &detail = "");
    std::string stateName(ApproachState s);
    static const char *navigationStatusName(int status);
    static bool isTerminalNavigationStatus(int status);
    void resetNavigationStatus();
    NavigationStatusSnapshot navigationStatusSnapshot();
    void navigationStatusCallback(
        const actionlib_msgs::msg::GoalStatus::SharedPtr msg);
    bool isObjectTarget(const std::string &category,
                        const std::string &id) const;
    bool applyObjectLateralAdjustment(
        const std::string &category,
        const std::string &id,
        const geometry_msgs::msg::PoseStamped &local_pose,
        const std::shared_ptr<GoalHandle> &gh,
        std::string &detail);
    bool moveBaseLaterally(double distance_m,
                           const std::shared_ptr<GoalHandle> &gh) const;
    // Detiene la navegación activa publicando la pose actual del robot como
    // nuevo goal. Go() es un tópico (no action), así que sobrescribir con
    // "ve a donde ya estás" hace que el nav_controller deje de comandar
    // velocidades de inmediato — equivalente funcional de Cancel().
    void stopNavigation();
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Hardware> hardware_;
    std::shared_ptr<Vision> vision_;
    std::shared_ptr<Knowledge> knowledge_;
    std::shared_ptr<Navigation> navigation_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<
        actionlib_msgs::msg::GoalStatus>::SharedPtr
            navigation_status_sub_;
    std::mutex navigation_status_mutex_;
    bool waiting_navigation_status_{false};
    bool seen_navigation_progress_{false};
    bool navigation_done_{false};
    bool navigation_success_{false};
    bool has_navigation_status_{false};
    int navigation_status_code_{-1};
    std::string navigation_status_text_;
    double object_lateral_offset_m_{0.15};
    double centered_y_threshold_m_{0.08};
    double person_partial_distance_m_{1.0};
    double person_partial_min_remaining_m_{0.75};
    double person_partial_timeout_s_{45.0};
};
#endif
