#include "executor_tools/Say.hpp"

Say::Say(const rclcpp::Node::SharedPtr& node,
         const std::shared_ptr<HRI>& hri)
: node(node),
  hri(hri),
  executor(node)
{
}

// ---------------------------------------------------------
// Leer valor salvado
// ---------------------------------------------------------
std::string Say::readSavedValue(const std::string& key)
{
    try
    {
        std::string package_path = ament_index_cpp::get_package_share_directory("knowledge_bank");
        std::string file_path = package_path + "/save/" + key + ".yaml";

        YAML::Node config = YAML::LoadFile(file_path);
        if (!config["phrases"] || config["phrases"].size() == 0)
        {
            RCLCPP_WARN(node->get_logger(), "No phrases in %s", file_path.c_str());
            return key;
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
                     "Failed to read saved value for %s: %s",
                     key.c_str(), e.what());
        return key;
    }
    catch (...)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Unknown error reading saved value for %s", key.c_str());
        return key;
    }
}

// ---------------------------------------------------------
// Reemplazar {key} con valor guardado
// ---------------------------------------------------------
std::string Say::replacePlaceholdersWithSaved(const std::string& text)
{
    std::string result = text;
    size_t start_pos = 0;

    while ((start_pos = result.find('{', start_pos)) != std::string::npos)
    {
        size_t end_pos = result.find('}', start_pos);
        if (end_pos == std::string::npos)
            break;

        std::string key = result.substr(start_pos + 1, end_pos - start_pos - 1);
        std::string value = readSavedValue(key);

        result.replace(start_pos, end_pos - start_pos + 1, value);
        start_pos += value.length();
    }

    return result;
}

// ---------------------------------------------------------
// Selecciona frase aleatoria
// ---------------------------------------------------------
std::string Say::getRandomPhrase(const std::string& action,
                                 const std::string& argument)
{
    try
    {
        std::string package_path = ament_index_cpp::get_package_share_directory("knowledge_bank");
        std::string file_path = package_path + "/" + action + "/" + argument + ".yaml";

        YAML::Node config = YAML::LoadFile(file_path);

        if (!config["phrases"] || config["phrases"].size() == 0)
        {
            RCLCPP_WARN(node->get_logger(),
                        "No phrases found in %s",
                        file_path.c_str());
            return argument;
        }

        auto phrases = config["phrases"];
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<> dist(0, phrases.size() - 1);

        return replacePlaceholdersWithSaved(phrases[dist(gen)].as<std::string>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to load phrase for %s: %s",
                     argument.c_str(), e.what());
        return "Error loading phrase";
    }
    catch (...)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Unknown error loading phrase for %s", argument.c_str());
        return "Error loading phrase";
    }
}

// ---------------------------------------------------------
// Ejecutar run
// ---------------------------------------------------------
bool Say::run(const std::string& text,
              const std::shared_ptr<GoalHandle>& goalHandle)
{
    std::string utterance = getRandomPhrase("say", text);

    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.step_id = 1;
    feedback->feedback.progress = 0.5f;
    feedback->feedback.message = "Saying: " + utterance;
    goalHandle->publish_feedback(feedback);

    bool success = hri->Say(utterance);
    std::string result = success ? "uttered: " + utterance : "tts_failed";

    return executor.authorize(
        "say",
        text,
        result,
        success ? "TTS executed" : "TTS failed"
    );
}

