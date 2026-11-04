cmake_minimum_required(VERSION 3.8)
project(workspace_setup)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install CycloneDDS configuration file
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Docker runtime assets
install(DIRECTORY docker/
  DESTINATION share/${PROJECT_NAME}/docker
)

# Register environment hooks
# This will execute workspace_setup.sh/dsv on workspace source
ament_environment_hooks(
  "env_hook/${PROJECT_NAME}.dsv.in"
  "env_hook/${PROJECT_NAME}.sh.in"
)

# Install installer & diagnostic scripts
install(PROGRAMS
  scripts/setup_env.sh
  scripts/verify_dds.sh
  scripts/install_docker.sh
  scripts/setup_apcupsd.sh
  scripts/setup_cyclonedds_sysctl.sh
  scripts/setup_workspace.sh
  scripts/build_image.sh
  scripts/start_ros2.sh
  scripts/enter_ros2.sh
  scripts/stop_ros2.sh
  scripts/status_ros2.sh
  scripts/reset_ros2_container.sh
  scripts/set_domain_id.sh
  scripts/lidar_control_center.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(control_center test/test_lidar_control_center.py TIMEOUT 60)
  ament_add_pytest_test(crane_integration test/test_crane_integration.py TIMEOUT 60)
endif()

ament_package()
