#ifndef EXECUTOR_TOOLS__APPROACH_HPP_
#define EXECUTOR_TOOLS__APPROACH_HPP_
#include <memory>
#include <mutex>
#include <string>
#include "actionlib_msgs/msg/goal_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
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
        bool collision_risk{false};
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
    void clearCollisionRiskFlag();
    NavigationStatusSnapshot navigationStatusSnapshot();
    void navigationStatusCallback(
        const actionlib_msgs::msg::GoalStatus::SharedPtr msg);
    void collisionRiskCallback(
        const std_msgs::msg::Bool::SharedPtr msg);
    void simpleMoveStatusCallback(
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
    bool applySafeDropFinalCorrection(
        const std::string &category,
        const geometry_msgs::msg::PoseStamped &global_pose,
        const std::shared_ptr<GoalHandle> &gh,
        std::string &detail);
    bool moveBaseRelative(double dx_m,
                          double dy_m,
                          const std::shared_ptr<GoalHandle> &gh) const;
    bool rotateBaseInPlace(double yaw_rad,
                           const std::shared_ptr<GoalHandle> &gh) const;
    bool confirmAndOrientToPerson(
        const std::string &category,
        const std::string &id,
        const geometry_msgs::msg::PoseStamped &global_pose,
        const std::shared_ptr<GoalHandle> &gh,
        bool require_visual_confirmation,
        std::string &detail);
    // Detiene la navegación activa publicando señales de paro en mvn_pln y
    // simple_move además de poner velocidad cero en la base.
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
    rclcpp::Subscription<
        std_msgs::msg::Bool>::SharedPtr
            collision_risk_sub_;
    rclcpp::Subscription<
        std_msgs::msg::Bool>::SharedPtr
            collision_risk_global_sub_;
    rclcpp::Subscription<
        actionlib_msgs::msg::GoalStatus>::SharedPtr
            simple_move_status_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr
        nav_stop_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr
        simple_move_stop_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        nav_ctrl_stop_pub_;
    std::mutex navigation_status_mutex_;
    bool waiting_navigation_status_{false};
    bool seen_navigation_progress_{false};
    bool navigation_done_{false};
    bool navigation_success_{false};
    bool has_navigation_status_{false};
    bool collision_risk_detected_{false};
    int navigation_status_code_{-1};
    std::string navigation_status_text_;
    double object_lateral_offset_m_{0.15};
    double centered_y_threshold_m_{0.08};
    double person_partial_distance_m_{1.0};
    double person_partial_min_remaining_m_{0.75};
    double person_partial_timeout_s_{45.0};
};
#endif
