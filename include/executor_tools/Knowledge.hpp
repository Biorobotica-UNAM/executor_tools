#pragma once
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <yaml-cpp/yaml.h> 
#include "geometry_msgs/msg/point_stamped.hpp"

class Knowledge {
public:
    Knowledge(const rclcpp::Node::SharedPtr &node) : node(node), last_recognized_("") {
        std::string package_share = ament_index_cpp::get_package_share_directory("knowledge_bank");
        std::string path = package_share + "/save/interest_words.yaml";
        
        RCLCPP_INFO(node->get_logger(), "Knowledge: Loading from: %s", path.c_str());
        loadInterestWords(path);
    }

    // --- MÉTODOS DE RAM (No se guardan en el YAML, son volátiles) ---
    
    // Lo llama el Find para guardar qué objeto vio exactamente (ej: "pepsi")
    void saveInRAM(const std::string &key, const std::string &value) {
        memory_ram_[key] = value;
        RCLCPP_INFO(node->get_logger(), "Knowledge RAM: [%s] = %s", key.c_str(), value.c_str());
    }

    std::string getFromRAM(const std::string &key) {
        if (memory_ram_.count(key)) return memory_ram_[key];
        return "unknown";
    }

    // Lo llama el Find para guardar LA POSICIÓN del objeto para el brazo
    void updateObjectPose(const std::string &item, const geometry_msgs::msg::PointStamped &pose) {
        object_poses_[item] = pose;
        RCLCPP_INFO(node->get_logger(), "Knowledge: Position for '%s' stored (x: %.2f)", 
                    item.c_str(), pose.point.x);
    }

    // Lo llama el Take para saber A DÓNDE MOVERSE
    geometry_msgs::msg::PointStamped getObjectPose(const std::string &item) {
        if (object_poses_.count(item)) return object_poses_[item];
        
        RCLCPP_WARN(node->get_logger(), "Knowledge: Pose for '%s' not found!", item.c_str());
        return geometry_msgs::msg::PointStamped(); // Devuelve una pose vacía si no existe
    }

    // --- MÉTODOS DE PERSISTENCIA (YAML) ---

    std::string getNameFromLabel(const std::string &label) {
        try {
            if (full_config_["labels"] && full_config_["labels"][label]) {
                return full_config_["labels"][label].as<std::string>();
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(node->get_logger(), "Knowledge: Label %s not found.", label.c_str());
        }
        return label; 
    }

    void loadInterestWords(const std::string &path) {
        try {
            full_config_ = YAML::LoadFile(path);
            std::vector<std::string> categories = {"names", "interests", "drinks"};

            for (const auto &cat : categories) {
                if (full_config_["categories"] && full_config_["categories"][cat]) {
                    for (const auto &word : full_config_["categories"][cat]) {
                        word_bank_[word.as<std::string>()] = cat;
                    }
                }
            }
            RCLCPP_INFO(node->get_logger(), "Knowledge: Word bank ready.");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node->get_logger(), "Knowledge: Error loading YAML: %s", e.what());
        }
    }

    void setLastRecognized(const std::string &phrase) {
        std::string lower_phrase = phrase;
        std::transform(lower_phrase.begin(), lower_phrase.end(), lower_phrase.begin(), ::tolower);
        std::stringstream ss(lower_phrase);
        std::string word;
        std::vector<std::string> found_words;

        while (ss >> word) {
            if (word_bank_.count(word)) {
                found_words.push_back(word);
            }
        }

        if (!found_words.empty()) {
            std::string result = "";
            for (size_t i = 0; i < found_words.size(); ++i) {
                result += found_words[i] + (i == found_words.size() - 1 ? "" : " ");
            }
            last_recognized_ = result;
        } else {
            last_recognized_ = "unknown";
        }
    }

    std::string getLastRecognized() const { return last_recognized_; }

    bool updateFromService(const std::string &label, const std::string &value) {
        try {
            full_config_["labels"][label] = value;
            return saveToFileDirect();
        } catch (const std::exception &e) {
            return false;
        }
    }

    bool saveToFileDirect() {
        std::string path = "src/Knowledge/knowledge_bank/save/interest_words.yaml";
        std::ofstream fout(path, std::ios::trunc);
        if (!fout.is_open()) return false;
        fout << full_config_; 
        fout.close();
        return true;
    }

    // Mantenemos saveToFile para compatibilidad con el Save anterior
    bool saveToFile(const std::string &labelName) {
        if (last_recognized_ != "unknown") {
            full_config_["labels"][labelName] = last_recognized_;
            return saveToFileDirect();
        }
        return false;
    }

    void updateLabel(const std::string &label, const std::string &value) {
        // Usamos updateFromService para que lo guarde en RAM y también en el archivo YAML
        updateFromService(label, value);
        RCLCPP_INFO(node->get_logger(), "Knowledge: Label [%s] updated to [%s]", 
                    label.c_str(), value.c_str());
    }

private:
    rclcpp::Node::SharedPtr node;
    YAML::Node full_config_;
    std::string last_recognized_;
    
    // NUEVAS ESTRUCTURAS DE DATOS PARA RAM
    std::map<std::string, std::string> memory_ram_;
    std::map<std::string, geometry_msgs::msg::PointStamped> object_poses_;
    
    std::map<std::string, std::string> word_bank_;
};