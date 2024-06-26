cmake_minimum_required(VERSION 3.5)
project(mini_pupper_description)

# Install the 'mini_pupper' and 'mini_pupper_2' directories
install(DIRECTORY
  mini_pupper
  mini_pupper_2
  DESTINATION share/${PROJECT_NAME}/config/champ
)