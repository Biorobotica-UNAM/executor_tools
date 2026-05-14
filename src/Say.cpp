#include "executor_tools/Say.hpp"

// =========================================================
// Archivos de frases — orden de búsqueda
// =========================================================
const std::vector<std::string> Say::PHRASE_FILES = {
    "navigation.yaml",
    "interaction.yaml",
    "introduction.yaml",
    "request.yaml",
    "confirmation.yaml",
    "identification.yaml",
    "description.yaml",
    "search.yaml",
    "challenges.yaml",
    "inquiry.yaml"
};

std::mt19937 Say::rng_(std::random_device{}());

// =========================================================
// Constructor
// =========================================================
Say::Say(const rclcpp::Node::SharedPtr& node,
         const std::shared_ptr<HRI>&    hri)
: node_(node), hri_(hri)
{}

// =========================================================
// run
// =========================================================
Result Say::run(const std::string&                 phrase_key,
                    const std::shared_ptr<GoalHandle>& gh)
{
    if (!gh || !gh->is_active())
        return {false, "FAILED", "goal_handle_inactive"};

    // ── READY ─────────────────────────────────────────────
    publishFeedback(gh, SayState::READY, phrase_key);

    // ── LOADING ───────────────────────────────────────────
    publishFeedback(gh, SayState::LOADING);

    std::string utterance = loadPhrase(phrase_key);

    if (utterance.empty()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Say: key '%s' not found — saying key as-is.",
                    phrase_key.c_str());
        utterance = phrase_key;
    }

    utterance = resolvePlaceholders(utterance, phrase_key);

    // ── SPEAKING ──────────────────────────────────────────
    publishFeedback(gh, SayState::SPEAKING, utterance);

    bool ok = hri_->Say(utterance);

    // ── SUCCESS / FAILED ──────────────────────────────────
    if (ok) {
        publishFeedback(gh, SayState::SUCCESS, utterance);
        return {true, "SUCCESS", "uttered: " + utterance};
    } else {
        publishFeedback(gh, SayState::FAILED, "tts_failed");
        return {false, "FAILED", "tts_failed: " + utterance};
    }
}

// =========================================================
// loadPhrase — busca la key en todos los archivos de phrases/
// =========================================================
std::string Say::loadPhrase(const std::string& key)
{
    std::string pkg;
    try {
        pkg = ament_index_cpp::get_package_share_directory("knowledge_bank");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Say: knowledge_bank not found: %s", e.what());
        return "";
    }

    const std::string base = pkg + "/phrases/";

    for (const auto& filename : PHRASE_FILES) {
        try {
            YAML::Node doc = YAML::LoadFile(base + filename);
            if (!doc[key] || !doc[key].IsSequence() || doc[key].size() == 0)
                continue;

            auto phrases = doc[key];
            std::uniform_int_distribution<size_t> dist(0, phrases.size() - 1);
            return phrases[dist(rng_)].as<std::string>();

        } catch (const std::exception&) {
            continue;
        }
    }

    return "";
}

// =========================================================
// resolvePlaceholders — reemplaza {key} con el argumento
// recibido del plan o con el nombre del key como fallback
// =========================================================
std::string Say::resolvePlaceholders(const std::string& text,
                                     const std::string& arg)
{
    // Por ahora el único valor disponible sin Knowledge es
    // el argumento que viene del plan. Se puede extender
    // cuando Knowledge vuelva a estar disponible.
    std::string result;
    result.reserve(text.size());

    static const std::regex re("\\{([^}]+)\\}");
    std::sregex_iterator it(text.begin(), text.end(), re);
    std::sregex_iterator end;
    size_t last = 0;

    for (; it != end; ++it) {
        const std::smatch& m = *it;
        result += text.substr(last, m.position() - last);

        // Intentamos resolver — de momento solo tenemos el arg del plan
        const std::string& placeholder = m[1].str();
        if (!arg.empty() && arg != text) {
            result += arg;
        } else {
            RCLCPP_WARN(node_->get_logger(),
                        "Say: placeholder {%s} not resolved.",
                        placeholder.c_str());
            result += "";
        }

        last = m.position() + m.length();
    }

    result += text.substr(last);
    return result;
}

// =========================================================
// publishFeedback
// =========================================================
void Say::publishFeedback(const std::shared_ptr<GoalHandle>& gh,
                          SayState                           state,
                          const std::string&                 detail)
{
    if (!gh->is_active()) return;

    std::string msg = "[say] " + stateName(state);
    if (!detail.empty()) msg += ": " + detail;

    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.message = msg;
    gh->publish_feedback(fb);

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

// =========================================================
// stateName
// =========================================================
std::string Say::stateName(SayState s)
{
    switch (s) {
        case SayState::READY:    return "READY";
        case SayState::LOADING:  return "LOADING";
        case SayState::SPEAKING: return "SPEAKING";
        case SayState::SUCCESS:  return "SUCCESS";
        case SayState::FAILED:   return "FAILED";
        default:                 return "UNKNOWN";
    }
}
