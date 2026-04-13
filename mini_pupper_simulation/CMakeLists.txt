cmake_minimum_required(VERSION 3.5)
project(mini_pupper_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  mini_pupper_simulation/smooth_position_test.py
  mini_pupper_simulation/odom_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch resource worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
