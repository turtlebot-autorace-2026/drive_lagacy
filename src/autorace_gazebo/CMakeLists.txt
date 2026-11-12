cmake_minimum_required(VERSION 3.8)
project(autorace_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(autorace_mission_devices_plugin SHARED src/mission_devices_plugin.cpp)
ament_target_dependencies(autorace_mission_devices_plugin gazebo_dev gazebo_ros rclcpp std_msgs sensor_msgs)

add_library(autorace_regions_plugin SHARED src/regions_plugin.cpp)
ament_target_dependencies(autorace_regions_plugin gazebo_dev gazebo_ros rclcpp std_msgs)

install(TARGETS autorace_mission_devices_plugin autorace_regions_plugin
  LIBRARY DESTINATION lib)
install(PROGRAMS scripts/race_manager.py scripts/camera_adapter.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch worlds models config
  DESTINATION share/${PROJECT_NAME})
install(FILES README_KO.md DESTINATION share/${PROJECT_NAME})

ament_package()
