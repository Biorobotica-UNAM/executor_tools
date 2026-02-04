cmake_minimum_required(VERSION 3.8)
project(executor_tools)

# Opciones de compilación estándar (por si luego metes C++)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Paquete base de CMake para ROS 2
find_package(ament_cmake REQUIRED)

# === Instalar archivos de launch (.launch.xml y .launch.py) ===
# Equivalente a:
# (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
# (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# === Instalar package.xml ===
# Equivalente a:
# ('share/' + package_name, ['package.xml'])
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# === Instalar resource/<package_name> ===
# Equivalente a:
# ('share/ament_index/resource_index/packages', ['resource/' + package_name])
#install(
#  FILES resource/${PROJECT_NAME}
#  DESTINATION share/ament_index/resource_index/packages
#)

ament_package()

