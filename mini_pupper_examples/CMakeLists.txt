cmake_minimum_required(VERSION 2.8.3)
project(mini_pupper_examples)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

file(GLOB python_scripts scripts/*.py)
catkin_install_python(
  PROGRAMS ${python_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
