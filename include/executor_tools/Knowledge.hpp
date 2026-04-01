#pragma once
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h> 

class Knowledge {
public:
    Knowledge(const rclcpp::Node::SharedPtr &node) : node(node), last_recognized_("") {
        std::string path = "src/Knowledge/knowledge_bank/save/interest_words.yaml";
        loadInterestWords(path);
    }

    std::string getNameFromLabel(const std::string &label) {
        try {
            // Buscamos en la nueva sección "labels" del YAML cargado
            if (full_config_["labels"] && full_config_["labels"][label]) {
                return full_config_["labels"][label].as<std::string>();
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(node->get_logger(), "Knowledge: Label %s not found. Error: %s", label.c_str(), e.what());
        }
        return label; // Si no existe, devolvemos el original (ej: "name_1")
    }

    void loadInterestWords(const std::string &path) {
        try {
            // IMPORTANTE: Cargamos directamente al miembro de la clase full_config_
            full_config_ = YAML::LoadFile(path);

            std::vector<std::string> categories = {"names", "interests", "drinks"};

            for (const auto &cat : categories) {
                if (full_config_["categories"] && full_config_["categories"][cat]) {
                    for (const auto &word : full_config_["categories"][cat]) {
                        word_bank_[word.as<std::string>()] = cat;
                    }
                }
            }
            RCLCPP_INFO(node->get_logger(), "Knowledge: Loaded %ld words and labels bank ready.", word_bank_.size());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node->get_logger(), "Knowledge: Error loading YAML: %s", e.what());
        }
    }

    //Lo llama listen
    void setLastRecognized(const std::string &phrase) {
        std::string lower_phrase = phrase;
        std::transform(lower_phrase.begin(), lower_phrase.end(), lower_phrase.begin(), ::tolower);

        std::stringstream ss(lower_phrase);
        std::string word;
        bool found = false;

        while (ss >> word) {
            if (word_bank_.count(word)) {
                last_recognized_ = word;
                last_category_ = word_bank_[word];
                
                RCLCPP_INFO(node->get_logger(), "Filter: Found '%s' in category '%s'", 
                            word.c_str(), last_category_.c_str());
                found = true;
                break; 
            }
        }

        if (!found) {
            last_recognized_ = "unknown";
            last_category_ = "none";
        }
    }

    // Lo llama la tool Save
    bool saveToFile(const std::string &labelName) {
        // 1. Actualizamos la memoria RAM en el nodo YAML completo
        // labelName sería "name_1", "drink_1", etc.
        if (last_recognized_ != "unknown") {
            // Esto actualiza la sección 'labels' en la variable full_config_
            full_config_["labels"][labelName] = last_recognized_;
            
            RCLCPP_INFO(node->get_logger(), "Knowledge: Linked label '%s' to '%s' in RAM.", 
                        labelName.c_str(), last_recognized_.c_str());
        } else {
            RCLCPP_WARN(node->get_logger(), "Knowledge: Attempted to save 'unknown' value to label '%s'. Skipping.", labelName.c_str());
            return false;
        }

        // 2. Persistencia en DISCO: Sobrescribimos el archivo MAESTRO
        std::string path = "src/Knowledge/knowledge_bank/save/interest_words.yaml";
        std::ofstream fout(path, std::ios::trunc);
        
        if (!fout.is_open()) {
            RCLCPP_ERROR(node->get_logger(), "Knowledge: Could not open MASTER file for saving: %s", path.c_str());
            return false;
        }

        // 3. Volcamos TODO el nodo YAML (incluyendo categories y los nuevos labels)
        fout << full_config_; 
        fout.close();

        RCLCPP_INFO(node->get_logger(), "Knowledge: Master file interest_words.yaml updated successfully.");
        return true;
    }

private:
    rclcpp::Node::SharedPtr node;
    YAML::Node full_config_;
    std::string last_recognized_;
    std::string last_category_; 
    std::map<std::string, std::string> word_bank_;
};