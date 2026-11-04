cmake_minimum_required(VERSION 3.8)
project(cx128s2_bev_perception)

# A default colcon build must retain real-time perception optimizations.
# Explicit Debug/RelWithDebInfo and multi-configuration generators are respected.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "cx128s2_bev_perception build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "msg/RecordingObjectCluster.msg"
  "msg/PerceptionRecordingFrame.msg"
  "msg/CraneBeamStatus.msg"
  "msg/CraneBeamMotionDebug.msg"
  "srv/CommitShapeMemory.srv"
  "srv/ControlShapeMemory.srv"
  "srv/PrepareShapeMemoryCommit.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

add_library(crane_beam_detector
  src/crane_beam_detector.cpp
  src/crane_hook_detector.cpp
)
target_link_libraries(crane_beam_detector yaml-cpp)
add_library(crane_pipeline src/crane_pipeline.cpp)
target_link_libraries(crane_pipeline crane_beam_detector Eigen3::Eigen)
target_include_directories(crane_pipeline PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(crane_pipeline tf2)
add_library(visibility_model src/visibility_model.cpp)
target_include_directories(visibility_model PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(visibility_model tf2)
target_link_libraries(crane_pipeline visibility_model)
target_include_directories(crane_beam_detector PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_executable(cx128s2_bev_perception_node
  src/cx128s2_bev_perception_node.cpp
)

ament_target_dependencies(cx128s2_bev_perception_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  std_srvs
  ament_index_cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(cx128s2_bev_perception_node "${cpp_typesupport_target}")
target_link_libraries(cx128s2_bev_perception_node crane_beam_detector)
target_link_libraries(cx128s2_bev_perception_node visibility_model crane_pipeline)
target_link_libraries(cx128s2_bev_perception_node yaml-cpp)
target_link_libraries(cx128s2_bev_perception_node nlohmann_json::nlohmann_json)
target_link_libraries(cx128s2_bev_perception_node OpenMP::OpenMP_CXX)
target_include_directories(cx128s2_bev_perception_node PRIVATE include)

# Crane V2 ships with the standard build; legacy/general remains available.
add_library(crane_v2_core src/v2/core.cpp)
target_include_directories(crane_v2_core PUBLIC include)
add_executable(crane_v2_experimental_node src/v2/experimental_node.cpp)
target_include_directories(crane_v2_experimental_node PRIVATE include)
ament_target_dependencies(crane_v2_experimental_node rclcpp sensor_msgs std_msgs
  geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(crane_v2_experimental_node crane_v2_core "${cpp_typesupport_target}")
set_target_properties(crane_v2_experimental_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

# Independent General V2: no legacy node or Crane detector dependency.
find_package(rcl_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_msgs REQUIRED)
add_library(general_v2_core
  src/general_v2/core.cpp src/general_v2/ground.cpp
  src/general_v2/detector.cpp src/general_v2/tracker.cpp)
target_include_directories(general_v2_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(general_v2_core Eigen3::Eigen)
file(GLOB GENERAL_V2_FINGERPRINT_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/general_v2/*.cpp" "src/general_v2/*.hpp"
  "include/cx128s2_bev_perception/general_v2/*.hpp")
list(APPEND GENERAL_V2_FINGERPRINT_FILES "include/cx128s2_bev_perception/vehicle_status.hpp")
list(SORT GENERAL_V2_FINGERPRINT_FILES)
set(GENERAL_V2_FINGERPRINT "")
foreach(source_file IN LISTS GENERAL_V2_FINGERPRINT_FILES)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${source_file}")
  file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/${source_file}" source_hash)
  string(APPEND GENERAL_V2_FINGERPRINT "${source_file}:${source_hash}\n")
endforeach()
string(SHA256 GENERAL_V2_SOURCE_SHA "${GENERAL_V2_FINGERPRINT}")
target_compile_definitions(general_v2_core PRIVATE GENERAL_V2_SOURCE_SHA="${GENERAL_V2_SOURCE_SHA}")
foreach(adapter IN ITEMS node replay)
  add_executable(general_v2_${adapter} src/general_v2/${adapter}.cpp)
  target_link_libraries(general_v2_${adapter} general_v2_core "${cpp_typesupport_target}" yaml-cpp nlohmann_json::nlohmann_json)
  ament_target_dependencies(general_v2_${adapter} rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs diagnostic_msgs tf2 tf2_ros std_srvs rosbag2_cpp tf2_msgs ament_index_cpp rcl_interfaces)
  target_compile_definitions(general_v2_${adapter} PRIVATE GENERAL_V2_SOURCE_SHA="${GENERAL_V2_SOURCE_SHA}")
endforeach()
install(TARGETS general_v2_core general_v2_node general_v2_replay
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS cx128s2_bev_perception_node crane_beam_detector visibility_model crane_pipeline
  crane_v2_core crane_v2_experimental_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/roi_bev_editor.py
  scripts/roi_bev_control_panel.py
  scripts/iir_track_benchmark.py
  scripts/kalman_filter_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  add_executable(test_general_v2_candidate test/test_general_v2_candidate.cpp)
  target_link_libraries(test_general_v2_candidate general_v2_core)
  add_test(NAME test_general_v2_candidate COMMAND test_general_v2_candidate)
  add_executable(test_general_ground_noise_filter test/test_general_ground_noise_filter.cpp)
  target_include_directories(test_general_ground_noise_filter PRIVATE include)
  add_test(NAME general_ground_noise_filter COMMAND test_general_ground_noise_filter)
  add_executable(test_general_v2_ros test/test_general_v2_ros.cpp)
  target_link_libraries(test_general_v2_ros general_v2_core "${cpp_typesupport_target}" yaml-cpp nlohmann_json::nlohmann_json)
  ament_target_dependencies(test_general_v2_ros rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs diagnostic_msgs tf2 tf2_ros std_srvs ament_index_cpp rcl_interfaces)
  add_test(NAME test_general_v2_ros COMMAND test_general_v2_ros --ros-args -p use_roi_policy:=false)
  set_tests_properties(test_general_v2_ros PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=174;ROS_LOCALHOST_ONLY=1")
  add_test(NAME test_general_v2_profile COMMAND test_general_v2_ros --validate-config --ros-args
    --params-file "${CMAKE_CURRENT_SOURCE_DIR}/config/cx128s2_bev_perception.yaml"
    --params-file "${CMAKE_CURRENT_SOURCE_DIR}/config/general_v2.yaml"
    -p "roi_policy_file:=${CMAKE_CURRENT_SOURCE_DIR}/config/cx128s2_roi_policy.yaml")
  set_tests_properties(test_general_v2_profile PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=175;ROS_LOCALHOST_ONLY=1")
  add_executable(test_general_v2 test/test_general_v2.cpp)
  target_link_libraries(test_general_v2 general_v2_core)
  add_test(NAME test_general_v2 COMMAND test_general_v2)
  add_executable(test_crane_v2_hist_motion test/test_crane_v2_hist_motion.cpp)
  target_include_directories(test_crane_v2_hist_motion PRIVATE include)
  add_test(NAME test_crane_v2_hist_motion COMMAND test_crane_v2_hist_motion)
  add_executable(test_crane_v2_retained_extent test/test_crane_v2_retained_extent.cpp)
  target_include_directories(test_crane_v2_retained_extent PRIVATE include)
  add_test(NAME test_crane_v2_retained_extent COMMAND test_crane_v2_retained_extent)
  add_executable(test_crane_v2_length_lock test/test_crane_v2_length_lock.cpp)
  target_link_libraries(test_crane_v2_length_lock crane_v2_core)
  add_test(NAME test_crane_v2_length_lock COMMAND test_crane_v2_length_lock)
  add_executable(crane_v2_replay EXCLUDE_FROM_ALL test/crane_v2_replay.cpp)
  target_link_libraries(crane_v2_replay crane_v2_core nlohmann_json::nlohmann_json)
  add_executable(test_crane_v2_shrink_hold test/test_crane_v2_shrink_hold.cpp)
  target_link_libraries(test_crane_v2_shrink_hold crane_v2_core)
  add_test(NAME test_crane_v2_shrink_hold COMMAND test_crane_v2_shrink_hold)
  add_executable(test_crane_v2_extent test/test_crane_v2_extent.cpp)
  target_link_libraries(test_crane_v2_extent crane_v2_core)
  add_test(NAME test_crane_v2_extent COMMAND test_crane_v2_extent)
  add_executable(test_crane_v2_ros test/test_crane_v2_ros.cpp)
  target_include_directories(test_crane_v2_ros PRIVATE include)
  ament_target_dependencies(test_crane_v2_ros rclcpp sensor_msgs std_msgs
    geometry_msgs visualization_msgs tf2 tf2_geometry_msgs)
  target_link_libraries(test_crane_v2_ros "${cpp_typesupport_target}")
  add_test(NAME test_crane_v2_ros COMMAND test_crane_v2_ros)
  add_executable(test_crane_v2 test/test_crane_v2.cpp)
  target_link_libraries(test_crane_v2 crane_v2_core)
  add_test(NAME test_crane_v2 COMMAND test_crane_v2)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_executable(test_tracking_policy test/test_tracking_policy.cpp)
  target_include_directories(test_tracking_policy PRIVATE include)
  # Includes the production translation unit to exercise private tracker paths.
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(test_tracking_policy PRIVATE -Wno-subobject-linkage)
  endif()
  ament_target_dependencies(test_tracking_policy
    rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs diagnostic_msgs
    tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs std_srvs ament_index_cpp)
  target_link_libraries(test_tracking_policy "${cpp_typesupport_target}"
    crane_beam_detector crane_pipeline visibility_model yaml-cpp OpenMP::OpenMP_CXX)
  add_test(NAME test_tracking_policy COMMAND test_tracking_policy --ros-args
    -p use_tf_transform:=false -p auto_load_background_on_start:=false
    -p use_roi_policy:=false -p visibility.enabled:=false
    -p publish_debug_topics:=true -p publish_recording_frame:=true
    -p auto_load_object_memory_bank_on_start:=false)
  add_executable(test_cargo_extension_bev test/test_cargo_extension_bev.cpp)
  target_include_directories(test_cargo_extension_bev PRIVATE include)
  # Includes the production translation unit to exercise private tracker paths.
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(test_cargo_extension_bev PRIVATE -Wno-subobject-linkage)
  endif()
  ament_target_dependencies(test_cargo_extension_bev
    rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs diagnostic_msgs
    tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs std_srvs ament_index_cpp)
  target_link_libraries(test_cargo_extension_bev "${cpp_typesupport_target}"
    crane_beam_detector crane_pipeline visibility_model yaml-cpp OpenMP::OpenMP_CXX)
  add_test(NAME test_cargo_extension_bev COMMAND test_cargo_extension_bev)
  add_executable(test_raw_vehicle_box_refinement test/test_raw_vehicle_box_refinement.cpp)
  target_include_directories(test_raw_vehicle_box_refinement PRIVATE include)
  add_test(NAME test_raw_vehicle_box_refinement COMMAND test_raw_vehicle_box_refinement)
  add_executable(test_linear_kalman_filter test/test_linear_kalman_filter.cpp)
  target_include_directories(test_linear_kalman_filter PRIVATE include)
  add_test(NAME test_linear_kalman_filter COMMAND test_linear_kalman_filter)
  add_executable(test_vehicle_status test/test_vehicle_status.cpp)
  target_include_directories(test_vehicle_status PRIVATE include)
  add_test(NAME test_vehicle_status COMMAND test_vehicle_status)
  add_executable(test_crane_beam_detector test/test_crane_beam_detector.cpp)
  target_link_libraries(test_crane_beam_detector crane_beam_detector)
  add_test(NAME test_crane_beam_detector COMMAND test_crane_beam_detector)
  add_executable(test_crane_pipeline test/test_crane_pipeline.cpp)
  target_link_libraries(test_crane_pipeline crane_pipeline crane_beam_detector nlohmann_json::nlohmann_json)
  add_test(NAME test_crane_pipeline COMMAND test_crane_pipeline)
  add_executable(test_crane_motion_policy test/test_crane_motion_policy.cpp)
  target_link_libraries(test_crane_motion_policy crane_pipeline crane_beam_detector nlohmann_json::nlohmann_json)
  add_test(NAME test_crane_motion_policy COMMAND test_crane_motion_policy)
  add_executable(test_vehicle_surface_tracking test/test_vehicle_surface_tracking.cpp)
  target_include_directories(test_vehicle_surface_tracking PRIVATE include)
  add_test(NAME test_vehicle_surface_tracking COMMAND test_vehicle_surface_tracking)
  add_executable(test_visibility_model test/test_visibility_model.cpp)
  target_link_libraries(test_visibility_model visibility_model)
  add_test(NAME test_visibility_model COMMAND test_visibility_model)
  # Assertions in regression fixtures include setup calls. Keep them active in
  # Release tests while leaving all production targets optimized with NDEBUG.
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    foreach(regression_target test_cargo_extension_bev test_tracking_policy test_raw_vehicle_box_refinement
        test_linear_kalman_filter test_vehicle_status test_crane_beam_detector
        test_crane_pipeline test_crane_motion_policy test_vehicle_surface_tracking test_visibility_model)
      target_compile_options(${regression_target} PRIVATE -UNDEBUG)
    endforeach()
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
