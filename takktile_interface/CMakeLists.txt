cmake_minimum_required(VERSION 2.8.3)
project(takktile_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  takktile_msgs
)

catkin_python_setup()

catkin_package(
CATKIN_DEPENDS
  takktile_msgs
)
