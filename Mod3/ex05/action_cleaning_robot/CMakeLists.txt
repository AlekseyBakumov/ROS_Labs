cmake_minimum_required(VERSION 3.8)
project(action_cleaning_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cleaning_action REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(cleaning_action_server src/cleaning_action_server.cpp)
add_executable(cleaning_action_client src/cleaning_action_client.cpp)

ament_target_dependencies(cleaning_action_server 
  rclcpp action_msgs geometry_msgs turtlesim rosidl_default_runtime cleaning_action rclcpp_action)
ament_target_dependencies(cleaning_action_client 
  rclcpp action_msgs geometry_msgs turtlesim rosidl_default_runtime cleaning_action rclcpp_action)

install(TARGETS
  cleaning_action_server
  cleaning_action_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
