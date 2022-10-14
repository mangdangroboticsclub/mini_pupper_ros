cmake_minimum_required(VERSION 2.8.3)
project(mini_pupper_navigation)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY config launch maps worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
