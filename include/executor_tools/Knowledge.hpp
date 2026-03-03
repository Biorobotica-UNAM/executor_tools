#pragma once
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <string>
#include <filesystem> 

class Knowledge {
public:
    Knowledge(const rclcpp::Node::SharedPtr &node) : node(node), last_recognized_("NoData") {}

    // Lo llama Listen
    void setLastRecognized(const std::string &data) {
        last_recognized_ = data;
    }

    // Lo llama Save
    bool saveToFile(const std::string &fileName) {
        std::string path = "src/Knowledge/knowledge_bank/save/" + fileName + ".yaml";
        std::ofstream fout(path, std::ios::trunc);
        if (!fout.is_open()) return false;

        fout << "value: \"" << last_recognized_ << "\"" << std::endl;
        fout.close();
        return true;
    }

private:
    rclcpp::Node::SharedPtr node;
    std::string last_recognized_; 
};