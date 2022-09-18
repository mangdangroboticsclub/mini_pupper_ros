cmake_minimum_required(VERSION 3.0.2)
project(display_interface)
find_package(catkin REQUIRED COMPONENTS 
  cv_bridge
  rospy
  sensor_msgs
)

catkin_package()
