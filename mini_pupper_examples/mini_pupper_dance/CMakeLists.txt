cmake_minimum_required(VERSION 2.8.3)
project(mini_pupper_dance)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY scripts launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
