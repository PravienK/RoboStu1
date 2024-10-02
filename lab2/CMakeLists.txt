cmake_minimum_required(VERSION 3.8)
project(lab2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# Add executables
add_executable(lidar_processor src/lab2.cpp)

# Link libraries
ament_target_dependencies(lidar_processor rclcpp sensor_msgs cv_bridge OpenCV)

# Install executables
install(TARGETS
  lidar_processor
  DESTINATION lib/${PROJECT_NAME})

ament_package()

