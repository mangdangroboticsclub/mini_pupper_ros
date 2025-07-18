cmake_minimum_required(VERSION 3.5)
project(mini_pupper_description)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Install data directories
install(DIRECTORY launch config rviz urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/stanford_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stanford_state_publisher
)

install(PROGRAMS
  src/stanford_joint_trajectory_to_states.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stanford_joint_trajectory_to_states
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
