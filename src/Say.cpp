#include "executor_tools/Say.hpp"

// =========================================================
// Constructor
// =========================================================
Say::Say(const rclcpp::Node::SharedPtr& node,
         const std::shared_ptr<HRI>& hri,
         const std::shared_ptr<Knowledge>& knowledge)
: node(node),
  hri(hri),
  knowledge(knowledge), // Guardamos la instancia compartida
  executor(node)
{
}

// =========================================================
// Lee valor guardado       interest_words.yaml -> labels
// =========================================================
std::string Say::readSavedValue(const std::string& key)
{
    std::string value = knowledge->getNameFromLabel(key);

    if (value == key) {
        RCLCPP_WARN(node->get_logger(), "Say: Key '{%s}' not found in Knowledge RAM", key.c_str());
    }

    return value;
}

// =========================================================
// Reemplaza {key} por el valor en labels 
// =========================================================
std::string Say::replacePlaceholdersWithSaved(const std::string& text)
{
    std::string result;
    std::regex re("\\{([^}]+)\\}");
    std::sregex_iterator begin(text.begin(), text.end(), re);
    std::sregex_iterator end;

    size_t last_pos = 0;
    for (auto it = begin; it != end; ++it)
    {
        std::smatch match = *it;
        result += text.substr(last_pos, match.position() - last_pos);

        std::string key = match[1].str();
        result += readSavedValue(key);

        last_pos = match.position() + match.length();
    }

    result += text.substr(last_pos);
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
        std::string file_path = package_path + "/say/phrases.yaml";

        YAML::Node config = YAML::LoadFile(file_path);

        if (!config[action_name] || !config[action_name].IsSequence() || config[action_name].size() == 0)
        {
            RCLCPP_WARN(node->get_logger(), "Category '%s' not found in phrases.yaml", action_name.c_str());
            return action_name; 
        }

        auto phrases = config[action_name];
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<> dist(0, phrases.size() - 1);

        return phrases[dist(gen)].as<std::string>();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error in getRandomPhrase: %s", e.what());
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

    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->feedback.message = "Diciendo: " + utterance;
    goalHandle->publish_feedback(feedback);

    bool success = hri->Say(utterance);

    return executor.authorize("say", action_name, 
                             success ? "uttered: " + utterance : "tts_failed",
                             success ? "TTS ejecutado" : "TTS fallido");
}