cmake_minimum_required(VERSION 3.0.2)
project(mini_pupper_control)

find_package(catkin REQUIRED COMPONENTS 
  cv_bridge
  rospy
  sensor_msgs
)

catkin_package()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

file(GLOB python_scripts scripts/*.py)
catkin_install_python(
  PROGRAMS ${python_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
