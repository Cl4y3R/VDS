cmake_minimum_required(VERSION 3.5)
project(chassis_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories("${CMAKE_CURRENT_LIST_DIR}/include/chassis_controller")

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lgsvl_msgs REQUIRED)

add_executable(chassis_controller src/chassis_controller.cpp)
ament_target_dependencies(chassis_controller message_filters rclcpp std_msgs sensor_msgs lgsvl_msgs) 

target_link_libraries(${PROJECT_NAME} m)

install(TARGETS
  chassis_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
