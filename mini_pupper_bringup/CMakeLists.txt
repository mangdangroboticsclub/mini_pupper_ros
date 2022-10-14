cmake_minimum_required(VERSION 3.0.2)
project(mini_pupper_bringup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
