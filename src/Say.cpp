#include "executor_tools/Say.hpp"

// =========================================================
// Constructor
// =========================================================
Say::Say(const rclcpp::Node::SharedPtr& node,
         const std::shared_ptr<HRI>& hri)
: node(node),
  hri(hri),
  executor(node)
{
}

// =========================================================
// Lee valor salvado
// =========================================================
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
        {
            RCLCPP_WARN(node->get_logger(),
                        "No 'value' found in %s, using filename as fallback",
                        file_path.c_str());
            return filename;
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to read saved value for %s: %s",
                     filename.c_str(),
                     e.what());
        return filename;
    }
}

// =========================================================
// Reemplaza {key} por sus values
// =========================================================
std::string Say::replacePlaceholdersWithSaved(const std::string& text)
{
    std::string result = text;
    std::regex re("\\{([^}]+)\\}");
    std::smatch match;

    auto searchStart = result.cbegin();
    while (std::regex_search(searchStart, result.cend(), match, re))
    {
        std::string key = match[1].str();
        std::string value = readSavedValue(key);

        result.replace(match.position(0), match.length(0), value);

        searchStart = result.cbegin() + match.position(0) + value.length();
    }

    return result;
}

// =========================================================
// Selecciona frase aleatoria
// =========================================================
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

// =========================================================
// Ejecuta say
// =========================================================
bool Say::run(const std::string& action_name,
              const std::shared_ptr<GoalHandle>& goalHandle)
{
    std::string utterance = getRandomPhrase(action_name);

    utterance = replacePlaceholdersWithSaved(utterance);

    // -------------------- FEEDBACK --------------------
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.step_id = 1;
    feedback->feedback.progress = 0.5f;
    feedback->feedback.message = "Diciendo: " + utterance;
    goalHandle->publish_feedback(feedback);

    // -------------------- HRI SAY --------------------
    bool success = hri->Say(utterance);

    std::string result =
        success ? "uttered: " + utterance : "tts_failed";

    // -------------------- AUTORIZACIÓN --------------------
    return executor.authorize(
        "say",
        action_name,
        result,
        success ? "TTS ejecutado" : "TTS fallido"
    );
}

