#include "executor_tools/Say.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <random>
#include <fstream>

Say::Say(const rclcpp::Node::SharedPtr& node,
         const std::shared_ptr<HRI>& hri)
: node(node),
  hri(hri),
  executor(node)
{
}

// ---------------------------------------------------------
// Get random phrase
// ---------------------------------------------------------

std::string Say::getRandomPhrase(
    const std::string& action,
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

    return phrases[dist(gen)].as<std::string>();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to load phrase for %s: %s",
                 argument.c_str(),
                 e.what());

    return argument;
  }
}

// ---------------------------------------------------------
// Run
// ---------------------------------------------------------

bool Say::run(
  const std::string& text,
  const std::shared_ptr<GoalHandle>& goalHandle)
{
  std::string utterance = getRandomPhrase("say", text);

  // -------------------- FEEDBACK --------------------
  auto feedback =
    std::make_shared<ExecuteTask::Feedback>();

  feedback->feedback.step_id = 1;
  feedback->feedback.progress = 0.5f;
  feedback->feedback.message =
    "Saying: " + utterance;

  goalHandle->publish_feedback(feedback);

  // -------------------- HRI SAY --------------------
  bool success = hri->Say(utterance);

  std::string result =
    success ? "uttered: " + utterance : "tts_failed";

  // -------------------- AUTHORIZE --------------------
  return executor.authorize(
    "say",
    text,               
    result,
    success ? "TTS executed" : "TTS failed"
  );
}

