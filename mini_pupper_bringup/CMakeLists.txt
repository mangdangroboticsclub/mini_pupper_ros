cmake_minimum_required(VERSION 3.5)
project(mini_pupper_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
