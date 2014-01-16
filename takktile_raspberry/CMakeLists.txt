cmake_minimum_required(VERSION 2.8.3)
project(takktile_raspberry)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  takktile_msgs
)

catkin_python_setup()

catkin_package(
CATKIN_DEPENDS
  takktile_msgs
)
