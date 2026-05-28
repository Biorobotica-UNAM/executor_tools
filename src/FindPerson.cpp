#include "executor_tools/FindPerson.hpp"

#include <chrono>
#include <thread>
#include <sstream>
#include <atomic>

using namespace std::chrono_literals;

// ============================================================
// Constructor
// ============================================================

FindPerson::FindPerson(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<Hardware>& hardware,
    const std::shared_ptr<Vision>& vision,
    const std::shared_ptr<Knowledge>& knowledge)
: node_(node),
  hardware_(hardware),
  vision_(vision),
  knowledge_(knowledge)
{}

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

    const bool anybody = argument.rfind("anybody", 0) == 0;

	std::string category;
	std::string person_index;

	if (anybody)
	{
		category = "guest";
		person_index = std::to_string(getNextGuestIndex());
	}
	else
	{
		const std::size_t sep = argument.rfind('_');

		if (sep == std::string::npos)
		{
		    category = argument;
		    person_index = "1";
		}
		else
		{
		    category     = argument.substr(0, sep);
		    person_index = argument.substr(sep + 1);
		}
	}

    // ── Scan positions ───────────────────────────────────────

    struct ScanPoint { double pan; double tilt; };
    std::vector<ScanPoint> scan_points;

    if (anybody) {
        // Tilt-only sweep: look straight ahead at multiple heights
        for (double tilt : {0.0, -0.2})
            scan_points.push_back({0.0, tilt});
    } else {
        // Pan sweep at neutral tilt to find a specific person
        for (double pan : {0.0, -0.6, 0.6, 0.0})
            scan_points.push_back({pan, 0.0});
    }

    // ── Scanning loop ────────────────────────────────────────

    for (const auto& sp : scan_points)
    {
        if (!gh->is_active())
            return {false, "FAILED", "goal_cancelled"};

        std::string found_name;

		const bool found = scanPosition(sp.pan, sp.tilt, anybody, argument, category, person_index, found_name,	gh);

        if (!gh->is_active())
            return {false, "FAILED", "goal_cancelled"};

        if (found) {
            publishFeedback(gh, FindPersonState::SUCCESS, found_name);
			return {true, "SUCCESS", "person_found: " + found_name};
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
    const std::string& category,
    const std::string& person_index,
    std::string& found_name,
    const std::shared_ptr<GoalHandle>& gh)
{
    // ── 1. Move head ──────────────────────────────────────────
    publishFeedback(gh, FindPersonState::SCANNING);
    hardware_->MoveHead({pan, tilt});

    for (int i = 0; i < 12; ++i) {
        std::this_thread::sleep_for(100ms);
        if (!gh->is_active()) return false;
    }

    // ── 2. Detect + pose ──────────────────────────────────────
	publishFeedback(gh, FindPersonState::DETECTING);

	Vision::PersonDetection detection;

	if (!vision_->DetectPersonPose(detection))
		return false;

	if (!gh->is_active())
		return false;

	geometry_msgs::msg::PoseStamped target_pose = detection.pose;

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

        found_name = identified_name;
        RCLCPP_INFO(node_->get_logger(),
                    "[FindPerson] Person identified: %s", found_name.c_str());

        // ── 6a. Describe ──────────────────────────────────────
        publishFeedback(gh, FindPersonState::DESCRIBING);

        std::string description;

		if (vision_->DescribePerson(found_name, description)) {
			RCLCPP_INFO(node_->get_logger(), "[FindPerson] Description: %s", description.c_str());
			publishFeedback(gh, FindPersonState::UPDATING, found_name);

			const auto payload = buildPersonPayload(category, person_index, found_name, target_pose, description, anybody);

			const bool stored =	knowledge_->update_world_state(payload);

			if (!stored)
			{
				publishFeedback(gh,	FindPersonState::FAILED, "update_world_state_failed");
				return false;
			}
		}	
		else {
			RCLCPP_WARN(node_->get_logger(), "[FindPerson] DescribePerson failed for '%s'", found_name.c_str());
		}
			
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
	const std::string new_name = generatePersonName(std::stoull(person_index));

    if (vision_->RegisterPerson(new_name)) {
        found_name = new_name;
        RCLCPP_INFO(node_->get_logger(),
                    "[FindPerson] Person registered as: %s", found_name.c_str());
    } else {
        RCLCPP_ERROR(node_->get_logger(),
                     "[FindPerson] Registration failed for '%s'", new_name.c_str());
        // Even if registration failed we still found somebody — report with
        // a fallback name so the caller knows a person was detected.
        found_name = new_name;
    }

    // ── 6b. Describe ─────────────────────────────────────────
    publishFeedback(gh, FindPersonState::DESCRIBING);

    std::string description;

	if (vision_->DescribePerson(found_name, description)) {
		RCLCPP_INFO(node_->get_logger(), "[FindPerson] Description: %s", description.c_str());
		publishFeedback(gh, FindPersonState::UPDATING, found_name);

			const auto payload = buildPersonPayload(category, person_index, found_name, target_pose, description, anybody);

			const bool stored =	knowledge_->update_world_state(payload);

			if (!stored)
			{
				publishFeedback(gh,	FindPersonState::FAILED, "update_world_state_failed");
				return false;
			}
	}
	else {
		RCLCPP_WARN(node_->get_logger(), "[FindPerson] DescribePerson failed for '%s'", found_name.c_str());
	}
    return true;   // anybody=true: any detected person counts as success
}

// ============================================================
// getNextGuestIndex
// ============================================================

uint64_t FindPerson::getNextGuestIndex()
{
    static std::atomic<uint64_t> counter{0};
    return ++counter;
}

// ============================================================
// generatePersonName
// ============================================================

std::string FindPerson::generatePersonName(uint64_t index)
{
    return "guest_" + std::to_string(index);
}

// ============================================================
// splitDescription
// ============================================================

std::vector<std::string> FindPerson::splitDescription(
    const std::string& description) const
{
    std::vector<std::string> result;

    std::stringstream ss(description);
    std::string item;

    while (std::getline(ss, item, ','))
    {
        item.erase(0, item.find_first_not_of(" "));
        item.erase(item.find_last_not_of(" ") + 1);

        result.push_back(item);
    }

    return result;
}

// ============================================================
// buildPersonPayload
// ============================================================

nlohmann::json FindPerson::buildPersonPayload(
    const std::string& category,
    const std::string& index,
    const std::string& name,
    const geometry_msgs::msg::PoseStamped& pose,
    const std::string& description,
    bool overwrite_name) const
{
    const auto tokens =
        splitDescription(description);

    std::string gender      = "unknown";
    std::string age_group   = "unknown";
    std::string emotion     = "unknown";
    std::string hair_color  = "unknown";
    std::string hair_style  = "unknown";
    std::string shirt_color = "unknown";

    std::vector<std::string> accessories;

    // ─────────────────────────────────────────────
    // gender
    // ─────────────────────────────────────────────

    if (tokens.size() > 0)
        gender = tokens[0];

    // ─────────────────────────────────────────────
	// shirt
	// ─────────────────────────────────────────────

	if (tokens.size() > 1)
	{
		const std::string& shirt = tokens[1];

		if (shirt.find("black") != std::string::npos ||
		    shirt.find("dark")  != std::string::npos)
		{
		    shirt_color = "black";
		}
		else if (shirt.find("blue") != std::string::npos)
		{
		    shirt_color = "blue";
		}
		else if (shirt.find("red") != std::string::npos)
		{
		    shirt_color = "red";
		}
		else if (shirt.find("white") != std::string::npos ||
		         shirt.find("light") != std::string::npos)
		{
		    shirt_color = "white";
		}
		else if (shirt.find("green") != std::string::npos)
		{
		    shirt_color = "green";
		}
		else if (shirt.find("yellow") != std::string::npos)
		{
		    shirt_color = "yellow";
		}
		else if (shirt.find("gray") != std::string::npos ||
		         shirt.find("grey") != std::string::npos)
		{
		    shirt_color = "gray";
		}
		else if (shirt.find("brown") != std::string::npos)
		{
		    shirt_color = "brown";
		}
		else if (shirt.find("orange") != std::string::npos)
		{
		    shirt_color = "orange";
		}
		else if (shirt.find("purple") != std::string::npos)
		{
		    shirt_color = "purple";
		}
		else if (shirt.find("pink") != std::string::npos)
		{
		    shirt_color = "pink";
		}
	}

    // ─────────────────────────────────────────────
    // age
    // ─────────────────────────────────────────────

    if (tokens.size() > 2)
    {
        if (tokens[2] == "young")
            age_group = "adult";
        else
            age_group = tokens[2];
    }

    // ─────────────────────────────────────────────
	// hair
	// ─────────────────────────────────────────────

	if (tokens.size() > 3)
	{
		const std::string& hair = tokens[3];

		// color
		if (hair.find("black") != std::string::npos ||
		    hair.find("dark")  != std::string::npos)
		{
		    hair_color = "black";
		}
		else if (hair.find("blonde") != std::string::npos ||
		         hair.find("yellow") != std::string::npos)
		{
		    hair_color = "blonde";
		}
		else if (hair.find("brown") != std::string::npos)
		{
		    hair_color = "brown";
		}
		else if (hair.find("white") != std::string::npos ||
		         hair.find("gray")  != std::string::npos ||
		         hair.find("grey")  != std::string::npos)
		{
		    hair_color = "gray";
		}
		else if (hair.find("red") != std::string::npos)
		{
		    hair_color = "red";
		}

		// style
		if (hair.find("short") != std::string::npos)
		{
		    hair_style = "short";
		}
		else if (hair.find("long") != std::string::npos)
		{
		    hair_style = "long";
		}
		else if (hair.find("curly") != std::string::npos)
		{
		    hair_style = "curly";
		}
		else if (hair.find("wavy") != std::string::npos)
		{
		    hair_style = "wavy";
		}
		else if (hair.find("bald") != std::string::npos)
		{
		    hair_style = "bald";
		}
	}

    // ─────────────────────────────────────────────
    // emotion
    // ─────────────────────────────────────────────

    if (tokens.size() > 4)
	{
		const std::string& emo = tokens[4];

		if (emo.find("happy") != std::string::npos ||
		    emo.find("smile") != std::string::npos)
		{
		    emotion = "happy";
		}
		else if (emo.find("sad") != std::string::npos)
		{
		    emotion = "sad";
		}
		else if (emo.find("angry") != std::string::npos)
		{
		    emotion = "angry";
		}
		else if (emo.find("serious") != std::string::npos ||
		         emo.find("neutral") != std::string::npos)
		{
		    emotion = "neutral";
		}
		else if (emo.find("surprised") != std::string::npos)
		{
		    emotion = "surprised";
		}
	}

    // ─────────────────────────────────────────────
    // accessories
    // ─────────────────────────────────────────────

    for (size_t i = 5; i < tokens.size(); ++i)
    {
        accessories.push_back(tokens[i]);
    }

    const auto& p = pose.pose.position;
    const auto& q = pose.pose.orientation;

    nlohmann::json person;

	if (overwrite_name)
		person["name"] = name;

	person["number"]      = std::stoi(index);
	person["gender"]      = gender;
	person["age_group"]   = age_group;
	person["emotion"]     = emotion;
	person["hair_color"]  = hair_color;
	person["hair_style"]  = hair_style;
	person["shirt_color"] = shirt_color;
	person["accessories"] = accessories;

    person["navigation"] = {
        p.x,
        p.y,
        p.z,
        q.x,
        q.y,
        q.z,
        q.w
    };

    return {
		{"category", category},
		{"person", person}
	};
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
		case FindPersonState::UPDATING:			return "UPDATING";
        case FindPersonState::SUCCESS:          return "SUCCESS";
        case FindPersonState::FAILED:           return "FAILED";
        default:                                return "UNKNOWN";
    }
}
