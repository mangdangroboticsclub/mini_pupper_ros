cmake_minimum_required(VERSION 3.5)
project(mini_pupper_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_interface REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  mini_pupper_simulation/smooth_position_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch resource worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  simple_quadruped_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

add_library(simple_quadruped_controller SHARED
  src/simple_quadruped_controller.cpp
)

target_include_directories(simple_quadruped_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(simple_quadruped_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

pluginlib_export_plugin_description_file(controller_interface simple_quadruped_controller_plugin.xml)

install(TARGETS simple_quadruped_controller
  EXPORT export_simple_quadruped_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_simple_quadruped_controller HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

ament_package()
