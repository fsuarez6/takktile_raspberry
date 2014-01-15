#! /usr/bin/env python
import rospy
import socket, struct
from takktile_raspberry import I2CInterface
# Messages
from takktile_raspberry.msg import Touch

class RobotiqTakktile():
  # Initialise
  def __init__(self):
    pass

if __name__ == '__main__':
  # Set up node
  rospy.init_node('robotiq_takktile', anonymous=True)
