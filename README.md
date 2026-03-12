# Executor Tools Package

## Description
The **`executor_tools`** package is a library of modular classes designed to handle the core capabilities of the **Justina** robot. Each "Tool" encapsulates a specific robotic behavior (navigation, perception, interaction) and manages the communication with the **Supervisor** through an authorization protocol.

This package provides the building blocks used by the `executor_worker` to perform complex tasks.

---

## Package Structure
The package follows a standard ROS 2 C++ structure, separating interface definitions (`include/`) from logic implementation (`src/`):

* **`Executor`**: Base class that handles the `AuthorizeTask` service calls to the Supervisor.
* **`Knowledge`**: Manages the local knowledge bank, including keyword filtering via YAML.
* **Behavioral Tools**: 
    * `GoTo`: Interface for the navigation system.
    * `Follow`: Human tracking activation.
    * `Listen` & `Say`: HRI (Human-Robot Interaction) utilities.
    * `Check`: Hardware status monitoring.
    * `Save`: Data persistence during tasks.

---

## Knowledge Management & Filtering
The `Knowledge` class allows the robot to recognize specific keywords from spoken phrases or instructions. It uses a YAML-based configuration for easy updates without recompilation.

### Keyword Categories
The system categorizes recognized words into:
* **Names**: Identifying people (e.g., "James", "Anthony").
* **Interests**: Recognizing hobbies or topics.
* **Drinks/Objects**: Categorizing items for manipulation tasks.

---

## Authorization Logic (`Executor` class)
Every tool utilizes the `Executor` class to synchronize with the **Supervisor**. This ensures that every action taken by the robot is logged and approved.

### `authorize` Method
```cpp
bool authorize(const std::string& action,
               const std::string& argument,
               const std::string& worker_result,
               const std::string& context_message);