cmake_minimum_required(VERSION 3.8)
project(diff_gps)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

find_library(SERIAL_LIB serial)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_library(diff_gps_component SHARED src/diff_gps_node.cpp)
ament_target_dependencies(diff_gps_component rclcpp_components rclcpp sensor_msgs geometry_msgs rtcm_msgs)
target_link_libraries(diff_gps_component ${SERIAL_LIB})

rclcpp_components_register_node(
    diff_gps_component
    PLUGIN "diff_gps::DiffGpsNode"
    EXECUTABLE diff_gps_node
)
# Install Cpp executables
ament_export_targets(export_diff_gps_component)
install(TARGETS diff_gps_component
        EXPORT export_diff_gps_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
