cmake_minimum_required(VERSION 3.5)
project(mini_pupper_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch maps param rviz
  DESTINATION share/${PROJECT_NAME}
)

<exec_depend>nav2_bringup</exec_depend>
<exec_depend>nav2_smac_planner</exec_depend>
<exec_depend>nav2_regulated_pure_pursuit_controller</exec_depend>
<exec_depend>nav2_velocity_smoother</exec_depend>

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
