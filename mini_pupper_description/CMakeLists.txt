cmake_minimum_required(VERSION 2.8.3)

project(mini_pupper_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

install(DIRECTORY config launch meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
