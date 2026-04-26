cmake_minimum_required(VERSION 3.14)
project(smart_assistive_system)

# ==============================================================================
# Smart Assistive System - CMake Build Configuration
# ==============================================================================
# Target: Raspberry Pi 4 (4GB) with ROS2 Humble
# Dependencies: OpenCV, ONNX Runtime, ROS2, wiringPi (GPIO)
# ==============================================================================

# C++17 standard required for ROS2 and modern features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization flags for Raspberry Pi 4 (ARM Cortex-A72)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -mcpu=cortex-a72 -mfpu=neon-fp-armv8")
  message(STATUS "Building for Raspberry Pi 4 (ARM Cortex-A72)")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
  message(STATUS "Building for generic architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Threading support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# ==============================================================================
# Find Dependencies
# ==============================================================================

# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# OpenCV
find_package(OpenCV 4.0 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# ONNX Runtime
find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATHS
    /usr/local/include/onnxruntime
    /usr/include/onnxruntime
    ${ONNXRUNTIME_ROOT}/include
)

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  PATHS
    /usr/local/lib
    /usr/lib
    ${ONNXRUNTIME_ROOT}/lib
)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
  message(STATUS "ONNX Runtime found: ${ONNXRUNTIME_LIB}")
else()
  message(WARNING "ONNX Runtime not found. Vision node will not perform inference.")
endif()

# wiringPi for GPIO (Raspberry Pi)
find_library(WIRINGPI_LIB wiringPi)
if(WIRINGPI_LIB)
  message(STATUS "wiringPi found: ${WIRINGPI_LIB}")
else()
  message(WARNING "wiringPi not found. GPIO-based nodes will use stub implementations.")
endif()

# espeak for TTS
find_library(ESPEAK_LIB espeak-ng)
if(NOT ESPEAK_LIB)
  find_library(ESPEAK_LIB espeak)
endif()
if(ESPEAK_LIB)
  message(STATUS "espeak found: ${ESPEAK_LIB}")
else()
  message(WARNING "espeak not found. Audio output will use system 'espeak' command.")
endif()

# ==============================================================================
# Define include directories
# ==============================================================================
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

# ==============================================================================
# Perception Layer
# ==============================================================================

# --- Vision Input Node ---
add_executable(vision_input_node
  src/perception_layer/vision_input.cpp
)
ament_target_dependencies(vision_input_node
  rclcpp std_msgs sensor_msgs cv_bridge image_transport
)
target_link_libraries(vision_input_node
  ${OpenCV_LIBS}
)
if(ONNXRUNTIME_LIB)
  target_link_libraries(vision_input_node ${ONNXRUNTIME_LIB})
  target_compile_definitions(vision_input_node PRIVATE HAS_ONNXRUNTIME)
endif()

# --- Spatial Input Node ---
add_executable(spatial_input_node
  src/perception_layer/spatial_input.cpp
)
ament_target_dependencies(spatial_input_node
  rclcpp std_msgs sensor_msgs geometry_msgs
)
if(WIRINGPI_LIB)
  target_link_libraries(spatial_input_node ${WIRINGPI_LIB})
  target_compile_definitions(spatial_input_node PRIVATE HAS_WIRINGPI)
endif()

# --- IMU Input Node ---
add_executable(imu_input_node
  src/perception_layer/imu_input.cpp
)
ament_target_dependencies(imu_input_node
  rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)
if(WIRINGPI_LIB)
  target_link_libraries(imu_input_node ${WIRINGPI_LIB})
  target_compile_definitions(imu_input_node PRIVATE HAS_WIRINGPI)
endif()

# --- Morse Input Node ---
add_executable(morse_input_node
  src/perception_layer/morse_input.cpp
)
ament_target_dependencies(morse_input_node
  rclcpp std_msgs
)
if(WIRINGPI_LIB)
  target_link_libraries(morse_input_node ${WIRINGPI_LIB})
  target_compile_definitions(morse_input_node PRIVATE HAS_WIRINGPI)
endif()

# ==============================================================================
# Cognitive Layer
# ==============================================================================

add_executable(cognitive_engine_node
  src/cognitive_layer/main.cpp
)
ament_target_dependencies(cognitive_engine_node
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs
)

# ==============================================================================
# Interaction Layer
# ==============================================================================

# --- Haptic Output Node ---
add_executable(haptic_output_node
  src/interaction_layer/haptic_output.cpp
)
ament_target_dependencies(haptic_output_node
  rclcpp std_msgs
)
if(WIRINGPI_LIB)
  target_link_libraries(haptic_output_node ${WIRINGPI_LIB})
  target_compile_definitions(haptic_output_node PRIVATE HAS_WIRINGPI)
endif()

# --- Audio Output Node ---
add_executable(audio_output_node
  src/interaction_layer/audio_output.cpp
)
ament_target_dependencies(audio_output_node
  rclcpp std_msgs
)
if(ESPEAK_LIB)
  target_link_libraries(audio_output_node ${ESPEAK_LIB})
  target_compile_definitions(audio_output_node PRIVATE HAS_ESPEAK)
endif()

# ==============================================================================
# Install targets
# ==============================================================================

install(TARGETS
  vision_input_node
  spatial_input_node
  imu_input_node
  morse_input_node
  cognitive_engine_node
  haptic_output_node
  audio_output_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  src/slam_mapping/launch
  DESTINATION share/${PROJECT_NAME}
)

# Install model files
install(DIRECTORY
  src/models
  DESTINATION share/${PROJECT_NAME}
)

# Install map files
install(DIRECTORY
  src/slam_mapping/maps
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# Ament package
# ==============================================================================
ament_package()
