cmake_minimum_required(VERSION 3.5)
project(mini_pupper_description)

# Install the 'champ' and 'ros_control' directories
install(DIRECTORY
  champ
  ros_control
  DESTINATION share/${PROJECT_NAME}/config
)