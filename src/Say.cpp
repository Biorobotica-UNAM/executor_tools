#include "executor_tools/Say.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <random>
#include <fstream>
#include <regex>
#include <sstream>

Say::Say(const rclcpp::Node::SharedPtr& node,
         const std::shared_ptr<HRI>& hri)
: node(node),
  hri(hri),
  executor(node)
{
}

// ---------------------------------------------------------
// Run
// ---------------------------------------------------------
bool Say::run(
  const std::string& action_name,
  const std::shared_ptr<GoalHandle>& goalHandle)
{
  std::string phrase = getRandomPhrase(action_name);

  std::string utterance = replacePlaceholdersWithSaved(phrase);

  // -------------------- FEEDBACK --------------------
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message = "Diciendo: " + utterance;
  goalHandle->publish_feedback(feedback);

  // -------------------- HRI SAY --------------------
  bool success = hri->Say(utterance);

  std::string result = success ? "uttered: " + utterance : "tts_failed";

  // -------------------- AUTHORIZE --------------------
  return executor.authorize(
    "say",
    utterance,
    result,
    success ? "TTS executed" : "TTS failed"
  );
}

// ---------------------------------------------------------
// Get random phrase 
// ---------------------------------------------------------
std::string Say::getRandomPhrase(const std::string& action_name)
{
  try
  {
    std::string package_path = ament_index_cpp::get_package_share_directory("knowledge_bank");
    std::string file_path = package_path + "/" + action_name + ".yaml";

    YAML::Node config = YAML::LoadFile(file_path);
    auto phrases = config["phrases"];
    if (!phrases || phrases.size() == 0)
      return "No phrases available";

    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, phrases.size() - 1);

    return phrases[dist(gen)].as<std::string>();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to load phrase for %s: %s",
                 action_name.c_str(), e.what());
    return "Error loading phrase";
  }
}

// ---------------------------------------------------------
// Replace all {} with saved values
// ---------------------------------------------------------
std::string Say::replacePlaceholdersWithSaved(std::string text)
{
  std::regex re("\\{([^}]+)\\}");
  std::smatch match;

  while (std::regex_search(text, match, re))
  {
    std::string key = match[1].str();
    std::string value = readSavedValue(key);
    text.replace(match.position(0), match.length(0), value);
  }

  return text;
}

// ---------------------------------------------------------
// Read saved value
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

    return filename;
  }
  catch (...)
  {
    return filename;
  }
}

