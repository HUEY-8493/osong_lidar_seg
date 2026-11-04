cmake_minimum_required(VERSION 3.8)
project(cx128s2_bev_data_recorder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cx128s2_bev_perception REQUIRED)

add_executable(cx128s2_bev_data_recorder_node src/cx128s2_bev_data_recorder_node.cpp)
ament_target_dependencies(cx128s2_bev_data_recorder_node
  rclcpp sensor_msgs std_srvs cx128s2_bev_perception)

install(TARGETS cx128s2_bev_data_recorder_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config tools DESTINATION share/${PROJECT_NAME})

ament_package()
