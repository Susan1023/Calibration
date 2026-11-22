# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025 Yao Yuzhuo (yaoyuzhuo6@gmail.com)

cmake_minimum_required(VERSION 3.8)
project(lidar_cam_validator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp CONFIG QUIET)  # VTK/PCL export on newer distributions
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

add_library(${PROJECT_NAME}
  src/calibration_validator.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  TBB::tbb
)

if(COMMAND ament_target_dependencies)
  ament_target_dependencies(${PROJECT_NAME}
    rclcpp ament_index_cpp sensor_msgs std_msgs cv_bridge message_filters pcl_conversions)
else()
  # ROS 2 distributions that removed the legacy ament_target_dependencies macro.
  target_link_libraries(${PROJECT_NAME}
    rclcpp::rclcpp ament_index_cpp::ament_index_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    cv_bridge::cv_bridge message_filters::message_filters
    pcl_conversions::pcl_conversions)
endif()

add_executable(validator_node src/validator_node.cpp)
target_link_libraries(validator_node ${PROJECT_NAME})
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(validator_node
    rclcpp sensor_msgs std_msgs cv_bridge message_filters)
else()
  target_link_libraries(validator_node
    rclcpp::rclcpp sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    cv_bridge::cv_bridge message_filters::message_filters)
endif()

install(TARGETS ${PROJECT_NAME} validator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/docs")
  install(DIRECTORY docs DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)
endif()

ament_package()
