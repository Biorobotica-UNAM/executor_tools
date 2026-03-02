#pragma once
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <string>
#include <filesystem> 

class Knowledge {
public:
    Knowledge(const rclcpp::Node::SharedPtr &node) : node(node) {}

    bool saveInfo(const std::string &data) {
        std::string path = "src/Knowledge/knowledge_bank/save/name_1.yaml";

        try {
            // 1. Abrir archivo. ios::trunc borra lo que existe 
            std::ofstream fout(path, std::ios::out | std::ios::trunc);
            
            if (!fout.is_open()) {
                RCLCPP_ERROR(node->get_logger(), "No se pudo abrir el archivo en: %s", path.c_str());
                return false;
            }

            // 2. Escribir el formato solicitado: value: "Nombre"
            fout << "value: \"" << data << "\"" << std::endl;
            fout.close();

            RCLCPP_INFO(node->get_logger(), "Datos guardados exitosamente en %s", path.c_str());
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node->get_logger(), "Excepción al guardar YAML: %s", e.what());
            return false;
        }
    }

private:
  rclcpp::Node::SharedPtr node;
};