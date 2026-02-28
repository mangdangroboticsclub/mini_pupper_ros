cmake_minimum_required(VERSION 3.5)
project(mini_pupper_simulation)

find_package(ament_cmake REQUIRED)
# ROS 2 Jazzy: Gazebo Classic → Gazebo Sim (Harmonic)
find_package(gz_ros2_control REQUIRED)
find_package(ros_gz_sim REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
