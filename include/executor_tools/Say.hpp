#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_tools/Result.hpp"

#include "hri_tools/HRI.hpp"
#include "knowledge_tools/Knowledge.hpp"

#include <string>
#include <memory>

// =========================================================
// FSM states
// =========================================================
enum class SayState {
    READY,
    LOADING,
    SPEAKING,
    SUCCESS,
    FAILED
};

class Say
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    Say(const rclcpp::Node::SharedPtr&    node,
        const std::shared_ptr<HRI>&       hri,
        const std::shared_ptr<Knowledge>& knowledge);

    /**
     * Execute the full say pipeline synchronously.
     *
     * @param phrase_key  Key identifying the phrase in the knowledge bank,
     *                    e.g. "start_hri", "acknowledge_object".
     *                    Passed directly to Knowledge::get_phrase().
     * @param gh          Active action goal handle for feedback publishing.
     * @return Result{true, "SUCCESS", "uttered: <text>"} on success.
     */
    Result run(const std::string&                 phrase_key,
               const std::shared_ptr<GoalHandle>& gh);

private:
    void publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                         SayState                           state,
                         const std::string&                 detail = "");

    static std::string stateName(SayState s);

    rclcpp::Node::SharedPtr    node_;
    std::shared_ptr<HRI>       hri_;
    std::shared_ptr<Knowledge> knowledge_;
};
