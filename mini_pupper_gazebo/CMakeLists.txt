cmake_minimum_required(VERSION 3.0.2)
project(mini_pupper_gazebo)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
