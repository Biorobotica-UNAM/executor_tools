# Executor Tools Package

## Proyecto Biorrobótica UNAM

## Description
Every tool utilizes the `Executor` class to synchronize with the **Supervisor**. This ensures that every action taken by the robot is logged and approved.

---

## Package Structure
The package follows a standard ROS 2 C++ structure, separating interface definitions (`include/`) from logic implementation (`src/`):

* **`Executor`**: Base class that handles the `AuthorizeTask` service calls to the Supervisor.
* **`Knowledge`**: Manages the local knowledge bank, including keyword filtering via YAML.
* **Behavioral Tools**:

### Supported Actions Reference Table

| Action | FSM / Class | Description |
| :--- | :--- | :--- |
| **`take`** | `Take` / `VisualGraspApproach` | Executes kinematic object grasping on tables or shelves by planning trajectories with MoveIt!. Generates frontal or top approach candidates, reorders joint sequence (shoulder/elbow → wrist → torso), applies closed-loop visual servoing correction using the mobile base, and retracts the arm by analytically reversing the trajectory. |
| **`verify_take`** | `VerifyTake` | Post-manipulation visual inspection using OWL-ViT v2. Compares the current scene against the previous pose registered in Knowledge; if the object is still detected on the surface, it clears the robot's hand assignment and outputs a grasp failure. |
| **`drop`** | `Drop` | Places objects on tables, shelves, containers, or on the floor (`floor`). Consumes the pose calculated by `FindSpace` or Knowledge, plans the approach with a fixed tool orientation ($q_y=-0.707$, $q_w=0.707$), opens the gripper, and retracts the arm while respecting exclusion prisms to avoid colliding with the released object. |
| **`find_space`** | `FindSpace` | Segments the RGB-D point cloud by enabling dedicated services for tables, containers, or shelves. Validates volumetric boundaries in `base_link`, discards stale transforms, and stores the `safe_drop/1` entity along with its global pose in `map` and its relative approach target. |
| **`give`** | `Give` | Transfers an object to a person. Identifies which hand holds the requested item, extends the arm forward, utters a voice countdown, opens the end-effector, waits 2 seconds for manual handoff, and updates hand availability in Knowledge. |
| **`receive`** | `Receive` | Receives an object from a user. Checks which arm is available (prioritizing the left one), extends the limb with an open gripper, verbally requests the delivery, closes the gripper after a pause, and updates object ownership in the world model. |
| **`find_object`** | `FindObject` | Sensory head sweep to locate objects or furniture. Supports queries for free seats ("seat") and open vocabulary detection using OWL-ViT v2. Centers the optical axis on the target via `hardware_->LookAt()` and updates manipulation (3D) and navigation (2D) coordinates. |
| **`find_person`** | `FindPerson` | Visual sweep for locating and tracking human faces. Actively stabilizes neck servos (`waitForHeadStable`), centers the face within the optical frame using closed-loop control, and interacts with the face recognizer to identify known users or dynamically register new guests (`guest_<id>`). |
| **`find_pose`** | `FindPose` | Detects human body poses (sitting, pointing, hand raised). Projects an approach pose scaled to 95% to avoid invading personal space and registers the coordinate into the shared buffer `customer_pose_map_` in the `map` frame. |
| **`align`** | `Align` | Proportional angular controller to align the robot base parallel to table edges or facing containers, requiring 3 consecutive samples below $1.5^\circ$ error. |
| **`focus`** | `Focus` | Toggles continuous visual tracking and gaze fixation within the vision subsystem using boolean flags. |
| **`analyze_objects`** | `AnalyzeObjects` | Applies visual reasoning to objects on tables. Implements cumulative counting (`count`), cardinal class comparison (`compare`), and filtering by size (`size`) or color (`color`), registering the selected instance's pose to enable immediate grasping via `Take`. |
| **`analyze_person`** | `AnalyzePerson` | Inspects people in the scene by evaluating clothing color, dynamic body gestures (`waving`, `pointing`, `raising`), or their combination, generating an approach pose toward the analyzed individual. |
| **`say`** | `Say` | Orchestrates Text-To-Speech (TTS) synthesis by extracting parameterized phrases from the Knowledge bank, resolving temporal variables (`day_of_week`) or guiding instructions, replacing underscores with natural pauses, and offering fallback support for plain text. |
| **`listen`** | `Listen` | Captures voice commands (ASR) in two modes: simple mode (direct keyword) and full mode (`<category>_<attribute>_<index>`), extracting semantic entities with up to 3 retries to register in Knowledge. |
| **`point`** | `Point` | Rotates the mobile base to absorb the head's pan angle while smoothing the neck to zero via exponential filtering ($\alpha=0.1$) and extends the left arm to point at a target. |
| **`check`** | `Check` | Checks physical environment conditions using hardware sensors, primarily door open or closed states (`CheckDoorState`). |
| **`follow`** | `Follow` | Toggles the closed-loop person tracking service managed by the HRI layer. |
| **`go_to`** | `GoTo` | Moves the base to predefined topological coordinates in the global map by invoking detailed navigation. |
| **`approach` / `approach_partial`** | `Approach` | Semantic approach toward people or objects. Projects local poses (`base_link`) to global ones (`map`), stops the kinematic controller when entering TF tolerance, applies residual correction for placement (`safe_drop`), and shifts the base laterally ($\pm0.15\text{ m}$) to align the arm performing the action. In partial mode, maintains a $0.75\text{ m}$ personal space distance from the user. |
| **`save_loc`** | `SaveLoc` | Registers topological locations in the map manager, intercepting special cases such as client return points, forward movements with a $+90^\circ$ angular offset (`kitchen_bar`), or initial calibration poses for the AMCL filter. |
| **`fold`** | `Fold` | Bimanual choreography for folding garments on tables. Identifies visual keypoints, sequentially folds both sleeves inward, grasps the bottom hem, and linearly advances the base ($0.15\text{ m}$ at $0.08\text{ m/s}$) to lay the bottom half over the garment. |

---

## ROS 2 Topics and Services

The `executor_tools` actions route their inputs, outputs, and synchronization across the following set of ROS 2 topics and services:

### Offered Services / Action Servers

* **`/executor/execute_task`** (`executor_interfaces/action/ExecuteTask`)
  * Main action server where each FSM reports micro-state feedback (`READY`, `APPROACHING`, `DETECTING`, etc.) and outputs the final structured result (`TaskResult`).

### Subscribed Topics

* **`/navigation/status`** (`actionlib_msgs/msg/GoalStatus`)
  * Monitored by the `Approach` node to detect goal arrival or navigation planner blockages.
* **`/tf`** and **`/tf_static`** (`tf2_msgs/msg/TFMessage`)
  * Consumed to resolve kinematics (`tf2_ros::Buffer`), relative Euclidean distances, and coordinate projections.

### Published Topics

* **`/nav_control/goal`** (`geometry_msgs/msg/PoseStamped`)
  * Issued via the `Navigation` facade to send navigation goals or force an immediate kinematic stop.
* **`/cmd_vel`** (`geometry_msgs/msg/Twist`)
  * Published by `Hardware` to command direct linear and angular velocities (`Align`, `Point`, `Drop`, `Fold`, `Approach`).

### Service Clients

* **`/vision/table_drop_tf/enable`** (`std_srvs/srv/SetBool`)
  * Enables flat table-space detection in `FindSpace`.
* **`/vision/container_drop_tf/enable`** (`std_srvs/srv/SetBool`)
  * Enables geometric container segmentation in `FindSpace`.
* **`/vision/shelf_drop_tf/enable`** (`std_srvs/srv/SetBool`)
  * Enables shelf level segmentation in `FindSpace`.
* **`/vision/owl/owlv2_service`** (`vision_interfaces/srv/GetDetections`)
  * Natural language-based open visual inference service used by `FindObject`, `VerifyTake`, `FindSpace`, and `Align`.
* **`/hri/human_tracking/enable`** (`std_srvs/srv/SetBool`)
  * Toggles the human tracking subsystem in `Follow`.
* **`/vision/container_drop_tf_node`** (`rcl_interfaces/srv/SetParameters`)
  * Dynamically adjusts release heights and Region of Interest (ROI) parameters at runtime in `FindSpace`.

---

## Knowledge Management & Filtering
The `Knowledge` class allows the robot to recognize specific keywords from spoken phrases or instructions. It uses a YAML-based configuration for easy updates without recompilation.

### Keyword Categories
The system categorizes recognized words into:
* **Names**: Identifying people (e.g., "James", "Anthony").
* **Interests**: Recognizing hobbies or topics.
* **Drinks/Objects**: Categorizing items for manipulation tasks.

---

### `authorize` Method
```cpp
bool authorize(const std::string& action,
               const std::string& argument,
               const std::string& worker_result,
               const std::string& context_message);
