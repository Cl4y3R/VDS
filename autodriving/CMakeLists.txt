cmake_minimum_required(VERSION 3.5)
project(autodriving)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lgsvl_msgs REQUIRED)

add_executable(main_msg src/msg_subpub.cpp)
ament_target_dependencies(main_msg message_filters rclcpp std_msgs sensor_msgs lgsvl_msgs) 

install(TARGETS
  main_msg
  DESTINATION lib/${PROJECT_NAME})

ament_package()