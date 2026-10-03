# Executor Tools Package

## Proyecto Biorrobótica UNAM

## Description
The **`executor_tools`** package is a library of modular classes designed to handle the core capabilities of the **Justina** robot. Each "Tool" encapsulates a specific robotic behavior (navigation, perception, interaction) and manages the communication with the **Supervisor** through an authorization protocol.

This package provides the building blocks used by the `executor_worker` to perform complex tasks.

---

## Package Structure
The package follows a standard ROS 2 C++ structure, separating interface definitions (`include/`) from logic implementation (`src/`):

* **`Executor`**: Base class that handles the `AuthorizeTask` service calls to the Supervisor.
* **`Knowledge`**: Manages the local knowledge bank, including keyword filtering via YAML.
* **Behavioral Tools**:

| Action | FSM | Description |
| :--- | :--- | :--- |
| **`say`** | `Say` | Pronuncia una frase del banco de conocimiento mediante el sintetizador de voz (TTS). |
| **`listen`** | `Listen` | Captura audio, extrae entidades nombradas y actualiza el estado del mundo. |
| **`go_to`** | `GoTo` | Ajusta la altura del torso y navega hacia una ubicación nombrada. |
| **`check`** | `Check` | Verifica el estado de un componente de hardware o sensor (p. ej., estado de una puerta). |
| **`point`** | `Point` | Alinea la base y la cabeza hacia un objetivo y extiende el brazo para señalarlo. |
| **`give`** | `Give` | Extiende el brazo que sostiene un objeto y abre el gripper tras confirmar la recepción. |
| **`receive`** | `Receive` | Extiende el brazo con el gripper abierto y lo cierra al detectar el objeto entregado por una persona. |
| **`take`** | `Take` | FSM principal de agarre. Ejecuta la aproximación visual fina mediante `VisualGraspApproach`, planea con `manipulation_tools`, cierra el gripper y retrae la trayectoria. |
| **`verify_take`** | `VerifyTake` | Confirma por visión la presencia del objeto tras la maniobra de agarre. |
| **`drop`** | `Drop` | Consume una TF de espacio seguro producida por `FindSpace` y deposita el objeto. |
| **`find_space`** | `FindSpace` | Localiza y valida una TF de espacio libre sobre superficies (mesas, repisas) para depositar objetos. |
| **`find_object`** | `FindObject` | Escanea con la cabeza, detecta objetos por visión y actualiza su pose en el mundo. |
| **`find_person`** | `FindPerson` | Detecta, localiza e identifica a una persona, actualizando su registro en el estado del mundo. |
| **`find_pose`** | `FindPose` | Detecta la postura de una persona y la almacena en un búfer compartido. |
| **`approach` / `approach_partial`** | `Approach` | Aproxima el robot a una persona u objeto. La variante `approach_partial` invoca `runPartial` para acercamientos incompletos. |
| **`align`** | `Align` | Alinea la base respecto a un objetivo visual mediante control en lazo cerrado. |
| **`focus`** | `Focus` | Centra la cámara de la cabeza en un objetivo previamente detectado. |
| **`follow`** | `Follow` | Habilita o deshabilita el servicio de seguimiento de personas. |
| **`analyze_objects`** | `AnalyzeObjects` | Realiza análisis de conteo, tamaño o color en objetos detectados. |
| **`analyze_person`** | `AnalyzePerson` | Analiza vestimenta o gestos de una persona. |
| **`save_loc`** | `SaveLoc` | Guarda la pose actual o relativa del robot como una nueva ubicación nombrada. |
| **`fold`** | `Fold` | Dobla una prenda sobre una mesa en dos fases (mangas y base) guiándose por puntos clave de visión. |

---

## Manipulacion: VisualGraspApproach

`Take.cpp` delega la estrategia fina de agarre a `VisualGraspApproach`.
La intencion es que `Take` conserve la maquina de estados de la accion, pero
que los detalles de pregrasp, candidatos, AprilTag/TF visual, avance final y
retract corto vivan en una clase aparte.

Flujo actual:

1. Elegir candidatos `FRONT` y dejar candidatos `TOP` preparados para cuando
   exista `PlanCartesianPath`.
2. Planear pregrasp con `manipulation_planner`, que usa MoveIt + OMPL +
   RRTConnect.
3. Ejecutar la trayectoria de pregrasp.
4. Intentar correccion visual `FRONT` usando el TF del marcador del gripper.
5. Si no hay TF visual disponible, mantener el fallback probado: avance fijo
   de base.
6. Guardar la distancia real avanzada para retroceder exactamente lo mismo.
7. Construir retract invirtiendo la trayectoria de pregrasp.

Parametros principales:

* `visual_grasp.use_visual_servo`
* `visual_grasp.use_base_final_approach`
* `visual_grasp.use_cartesian_final_approach`
* `visual_grasp.base_final_approach_distance_m`
* `visual_grasp.base_final_approach_speed_mps`
* `visual_grasp.visual_max_attempts`
* `visual_grasp.visual_max_total_correction_m`
* `visual_grasp.right_marker_frame`
* `visual_grasp.left_marker_frame`
* `visual_grasp.right_marker_to_tcp_xyz`
* `visual_grasp.left_marker_to_tcp_xyz`

Mientras `PlanCartesianPath` no exista, los candidatos `TOP` se omiten para no
intentar un agarre superior usando solo la base. El robot normalmente queda a
unos 30-50 cm de la mesa; por eso el fallback de base se mantiene parametrico y
limitado para probar avances finales sin recompilar.

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
