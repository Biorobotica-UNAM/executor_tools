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

    void loadInterestWords(const std::string &path) {
        try {
            YAML::Node config = YAML::LoadFile(path);

            std::vector<std::string> categories = {"names", "interests", "drinks"};

            for (const auto &cat : categories) {
                if (config["categories"][cat]) {
                    for (const auto &word : config["categories"][cat]) {
                        // Guardamos: word -> category (ej: "coke" -> "drinks")
                        word_bank_[word.as<std::string>()] = cat;
                    }
                }
            }
            RCLCPP_INFO(node->get_logger(), "Knowledge: Loaded %ld words from YAML.", word_bank_.size());
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

    //Lo llama save
    bool saveToFile(const std::string &fileName) {
        std::string path = "src/Knowledge/knowledge_bank/save/" + fileName + ".yaml";
        std::ofstream fout(path, std::ios::trunc);
        if (!fout.is_open()) return false;

        fout << "value: \"" << last_recognized_ << "\"" << std::endl;
        fout << "category: \"" << last_category_ << "\"" << std::endl;
        fout.close();
        return true;
    }

private:
    rclcpp::Node::SharedPtr node;
    std::string last_recognized_;
    std::string last_category_; // Nueva variable para saber qué tipo de palabra es
    std::map<std::string, std::string> word_bank_; // Memoria de palabras de interés
};