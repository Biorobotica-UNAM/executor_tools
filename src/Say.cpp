#include "executor_tools/Say.hpp"

Say::Say(const rclcpp::Node::SharedPtr& node,
         const std::shared_ptr<HRI>& hri)
: node(node),
  hri(hri),
  executor(node)
{
}

// ---------------------------------------------------------
// Read saved values
// ---------------------------------------------------------
std::string Say::readSavedValue(const std::string& filename)
{
    try
    {
        std::string package_path = ament_index_cpp::get_package_share_directory("knowledge_bank");
        std::string file_path = package_path + "/save/" + filename + ".yaml";

        YAML::Node config = YAML::LoadFile(file_path);
        if (config["value"])
            return config["value"].as<std::string>();
        else
            return filename;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to load saved value from %s: %s",
                     filename.c_str(),
                     e.what());
        return filename;
    }
}

// ---------------------------------------------------------
// Replace placeholders dynamically
// ---------------------------------------------------------
std::string Say::replacePlaceholdersWithSaved(const std::string& text)
{
    std::string result = text;
    std::regex placeholder_re("\\{([^}]+)\\}");
    std::smatch match;

    std::string::const_iterator searchStart(result.cbegin());
    while (std::regex_search(searchStart, result.cend(), match, placeholder_re))
    {
        std::string key = match[1].str();
        std::string value;

        if (key == "guest_1") value = readSavedValue("name_1");
        else if (key == "guest_2") value = readSavedValue("name_2");
        else if (key == "drink_1") value = readSavedValue("drink_1");
        else if (key == "drink_2") value = readSavedValue("drink_2");
        else value = key;

        result.replace(match.position(0), match.length(0), value);
        searchStart = result.cbegin() + match.position(0) + value.length();
    }

    return result;
}

// ---------------------------------------------------------
// Get a random phrase
// ---------------------------------------------------------
std::string Say::getRandomPhrase(const std::string& action_name)
{
    try
    {
        std::string package_path = ament_index_cpp::get_package_share_directory("knowledge_bank");
        std::string file_path = package_path + "/say/" + action_name + ".yaml";

        YAML::Node config = YAML::LoadFile(file_path);

        if (!config["phrases"] || config["phrases"].size() == 0)
        {
            RCLCPP_WARN(node->get_logger(),
                        "No phrases found in %s",
                        file_path.c_str());
            return action_name;
        }

        auto phrases = config["phrases"];
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<> dist(0, phrases.size() - 1);

        return phrases[dist(gen)].as<std::string>();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to load phrase for %s: %s",
                     action_name.c_str(),
                     e.what());
        return action_name;
    }
}

// ---------------------------------------------------------
// Run say action
// ---------------------------------------------------------
bool Say::run(
    const std::string& action_name,
    const std::shared_ptr<GoalHandle>& goalHandle)
{
    std::string utterance = getRandomPhrase(action_name);

    utterance = replacePlaceholdersWithSaved(utterance);

    // -------------------- FEEDBACK --------------------
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.step_id = 1;
    feedback->feedback.progress = 0.5f;
    feedback->feedback.message = "Saying: " + utterance;
    goalHandle->publish_feedback(feedback);

    // -------------------- HRI SAY --------------------
    bool success = hri->Say(utterance);

    std::string result =
        success ? "uttered: " + utterance : "tts_failed";

    // -------------------- AUTHORIZE --------------------
    return executor.authorize(
        "say",
        action_name,
        result,
        success ? "TTS executed" : "TTS failed"
    );
}

