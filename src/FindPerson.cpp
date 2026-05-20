#include "executor_tools/FindPerson.hpp"

#include <atomic>
#include <chrono>
#include <sstream>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// ============================================================
// Constructor
// ============================================================

FindPerson::FindPerson(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<Hardware>& hardware,
    const std::shared_ptr<Vision>& vision)
: node_(node),
  hardware_(hardware),
  vision_(vision)
{
    bbox_to_pose_client_ =
        node_->create_client<vision_interfaces::srv::BboxToPose>(
            "/vision/bbox_to_pose",
            rmw_qos_profile_services_default,
            vision_->getCallbackGroup());  // reuse Vision's registered Reentrant group
}

// ============================================================
// run
// ============================================================

Result FindPerson::run(
    const std::string& argument,
    const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    publishFeedback(gh, FindPersonState::READY, argument);

    const bool anybody = (argument == "anybody");

    // ── Scan positions ───────────────────────────────────────

    struct ScanPoint { double pan; double tilt; };
    std::vector<ScanPoint> scan_points;

    if (anybody) {
        // Tilt-only sweep: look straight ahead at multiple heights
        for (double tilt : {0.45, 0.25, 0.0, -0.2})
            scan_points.push_back({0.0, tilt});
    } else {
        // Pan sweep at neutral tilt to find a specific person
        for (double pan : {-1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2})
            scan_points.push_back({pan, 0.0});
    }

    // ── Scanning loop ────────────────────────────────────────

    for (const auto& sp : scan_points)
    {
        if (!gh->is_active())
            return {false, "FAILED", "goal_cancelled"};

        PersonData data;
        const bool found = scanPosition(
            sp.pan, sp.tilt, anybody, argument, data, gh);

        if (!gh->is_active())
            return {false, "FAILED", "goal_cancelled"};

        if (found) {
            publishFeedback(gh, FindPersonState::SUCCESS, data.name);
            return {true, "SUCCESS", "person_found: " + data.name};
        }
    }

    // ── Nobody found after full sweep ────────────────────────

    hardware_->MoveHead({0.0, 0.0});
    publishFeedback(gh, FindPersonState::FAILED, argument);
    return {false, "FAILED", "person_not_found: " + argument};
}

// ============================================================
// scanPosition
//
// Flow:
//   1. Move head to (pan, tilt)
//   2. DetectPerson  → any person visible?
//   3. BboxToPose    → get 3-D position
//   4. LookAt        → center camera on person
//   5. IdentifyPerson → is this person in the DB?
//      anybody=true  : always proceed to describe/register
//      anybody=false : only proceed if identified name == target
//   6a. Identified   → DescribePerson → return true
//   6b. Unknown      → RegisterPerson → DescribePerson → return (anybody)
// ============================================================

bool FindPerson::scanPosition(
    double pan,
    double tilt,
    bool anybody,
    const std::string& target,
    PersonData& data,
    const std::shared_ptr<GoalHandle>& gh)
{
    // ── 1. Move head ──────────────────────────────────────────
    publishFeedback(gh, FindPersonState::SCANNING);
    hardware_->MoveHead({pan, tilt});

    for (int i = 0; i < 12; ++i) {
        std::this_thread::sleep_for(100ms);
        if (!gh->is_active()) return false;
    }

    // ── 2. Detect ─────────────────────────────────────────────
    publishFeedback(gh, FindPersonState::DETECTING);

    // Pass empty name: server detects any visible person
    if (!vision_->DetectPerson("", data.coords, data.z))
        return false;

    if (!gh->is_active()) return false;

    // ── 3. BboxToPose ─────────────────────────────────────────
    publishFeedback(gh, FindPersonState::CALCULATING_POSE);

    geometry_msgs::msg::PoseStamped target_pose;
    if (!bboxToPose(data, target_pose)) {
        RCLCPP_WARN(node_->get_logger(),
                    "[FindPerson] BboxToPose failed — skipping position");
        return false;
    }

    if (!gh->is_active()) return false;

    // ── 4. Look at the person ─────────────────────────────────
    publishFeedback(gh, FindPersonState::LOOK_AT_TARGET);

    geometry_msgs::msg::PointStamped look_point;
    look_point.header = target_pose.header;
    look_point.point  = target_pose.pose.position;
    hardware_->LookAt(look_point);

    for (int i = 0; i < 15; ++i) {
        std::this_thread::sleep_for(100ms);
        if (!gh->is_active()) return false;
    }

    // ── 5. Identify ───────────────────────────────────────────
    publishFeedback(gh, FindPersonState::IDENTIFYING);

    std::string identified_name;
    const bool identified = vision_->IdentifyPerson(identified_name);

    if (identified)
    {
        // ── 5a. Person is in the DB ───────────────────────────

        if (!anybody && identified_name != target)
        {
            // Found someone, but not who we're looking for — skip
            RCLCPP_INFO(node_->get_logger(),
                        "[FindPerson] Identified '%s', not target '%s' — skipping",
                        identified_name.c_str(), target.c_str());
            return false;
        }

        data.name = identified_name;
        RCLCPP_INFO(node_->get_logger(),
                    "[FindPerson] Person identified: %s", data.name.c_str());

        // ── 6a. Describe ──────────────────────────────────────
        publishFeedback(gh, FindPersonState::DESCRIBING);

        if (vision_->DescribePerson(data.name, data.description))
            RCLCPP_INFO(node_->get_logger(),
                        "[FindPerson] Description: %s", data.description.c_str());
        else
            RCLCPP_WARN(node_->get_logger(),
                        "[FindPerson] DescribePerson failed for '%s'",
                        data.name.c_str());

        return true;   // success for both anybody and named target
    }

    // ── 5b. Person NOT in the DB ──────────────────────────────

    if (!anybody)
    {
        // Looking for a specific person; unknown faces are irrelevant
        RCLCPP_INFO(node_->get_logger(),
                    "[FindPerson] Unknown person at this position — skipping");
        return false;
    }

    // anybody=true: register the unknown person, then describe
    publishFeedback(gh, FindPersonState::REGISTERING);
    const std::string new_name = generatePersonName();

    if (vision_->RegisterPerson(new_name)) {
        data.name = new_name;
        RCLCPP_INFO(node_->get_logger(),
                    "[FindPerson] Person registered as: %s", data.name.c_str());
    } else {
        RCLCPP_ERROR(node_->get_logger(),
                     "[FindPerson] Registration failed for '%s'", new_name.c_str());
        // Even if registration failed we still found somebody — report with
        // a fallback name so the caller knows a person was detected.
        data.name = new_name;
    }

    // ── 6b. Describe ─────────────────────────────────────────
    publishFeedback(gh, FindPersonState::DESCRIBING);

    if (vision_->DescribePerson(data.name, data.description))
        RCLCPP_INFO(node_->get_logger(),
                    "[FindPerson] Description: %s", data.description.c_str());
    else
        RCLCPP_WARN(node_->get_logger(),
                    "[FindPerson] DescribePerson failed for '%s'",
                    data.name.c_str());

    return true;   // anybody=true: any detected person counts as success
}

// ============================================================
// bboxToPose
// ============================================================

bool FindPerson::bboxToPose(
    const PersonData& data,
    geometry_msgs::msg::PoseStamped& pose_out)
{
    int32_t x1{0}, y1{0}, x2{0}, y2{0};

    if (!data.coords.empty())
    {
        try {
            std::istringstream ss(data.coords);
            std::string tok;
            std::vector<int32_t> vals;
            while (std::getline(ss, tok, ','))
                vals.push_back(static_cast<int32_t>(std::stoi(tok)));

            if (vals.size() >= 4) {
                x1 = vals[0]; y1 = vals[1];
                x2 = vals[2]; y2 = vals[3];
            } else {
                RCLCPP_WARN(node_->get_logger(),
                            "[FindPerson] bboxToPose: incomplete coords '%s'",
                            data.coords.c_str());
                return false;
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(),
                         "[FindPerson] bboxToPose: parse error '%s': %s",
                         data.coords.c_str(), e.what());
            return false;
        }
    }
    else if (data.z <= 0.f)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[FindPerson] bboxToPose: no coords and no depth");
        return false;
    }

    if (!bbox_to_pose_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[FindPerson] /vision/bbox_to_pose service not available");
        return false;
    }

    auto request =
        std::make_shared<vision_interfaces::srv::BboxToPose::Request>();
    request->header.stamp    = node_->now();
    request->header.frame_id = "head_front_camera_rgb_optical_frame";
    request->x1 = x1;  request->y1 = y1;
    request->x2 = x2;  request->y2 = y2;

    // Use callback-based async_send_request (same pattern as Vision::call_service)
    // to avoid blocking the executor thread.
    auto prom =
        std::make_shared<std::promise<
            vision_interfaces::srv::BboxToPose::Response::SharedPtr>>();
    auto fut = prom->get_future();

    bbox_to_pose_client_->async_send_request(
        request,
        [prom](rclcpp::Client<vision_interfaces::srv::BboxToPose>::SharedFuture f) {
            prom->set_value(f.get());
        });

    if (fut.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[FindPerson] bboxToPose: timeout after 5 s");
        return false;
    }

    auto response = fut.get();
    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[FindPerson] bboxToPose: service returned failure");
        return false;
    }

    pose_out = response->pose;
    RCLCPP_INFO(node_->get_logger(),
                "[FindPerson] bboxToPose: (%.2f, %.2f, %.2f)",
                pose_out.pose.position.x,
                pose_out.pose.position.y,
                pose_out.pose.position.z);
    return true;
}

// ============================================================
// generatePersonName
// ============================================================

std::string FindPerson::generatePersonName()
{
    static std::atomic<uint64_t> counter{0};
    return "Person_" + std::to_string(++counter);
}

// ============================================================
// publishFeedback
// ============================================================

void FindPerson::publishFeedback(
    const std::shared_ptr<GoalHandle>& gh,
    FindPersonState state,
    const std::string& detail)
{
    if (!gh || !gh->is_active())
        return;

    std::string msg = "[find_person] " + stateName(state);
    if (!detail.empty())
        msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// ============================================================
// stateName
// ============================================================

std::string FindPerson::stateName(FindPersonState s)
{
    switch (s)
    {
        case FindPersonState::READY:            return "READY";
        case FindPersonState::SCANNING:         return "SCANNING";
        case FindPersonState::DETECTING:        return "DETECTING";
        case FindPersonState::CALCULATING_POSE: return "CALCULATING_POSE";
        case FindPersonState::LOOK_AT_TARGET:   return "LOOK_AT_TARGET";
        case FindPersonState::IDENTIFYING:      return "IDENTIFYING";
        case FindPersonState::REGISTERING:      return "REGISTERING";
        case FindPersonState::DESCRIBING:       return "DESCRIBING";
        case FindPersonState::SUCCESS:          return "SUCCESS";
        case FindPersonState::FAILED:           return "FAILED";
        default:                                return "UNKNOWN";
    }
}