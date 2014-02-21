#! /usr/bin/env python
import rospy, math
import numpy as np
# Sockets
import socket, struct
from cStringIO import StringIO
# TakkStrip
import takktile_raspberry as rpi

# Messages
from takktile_msgs.msg import RobotiqTouch

class RobotiqUDP():
  # Initialise
  def __init__(self, topic):
    self.ns = rospy.get_namespace()
    # Set up sender socket
    self.write_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.write_ip = rospy.get_param('~write_ip', '127.0.0.1')
    self.write_port = int(rospy.get_param('~write_port', 5050))
    step = int(rospy.get_param('~sensors_per_finger', 4))
    rospy.loginfo('UDP Socket sending to [udp://%s:%d]' % (self.write_ip, self.write_port))
    # Set-up publishers/subscribers
    touch_pub = rospy.Publisher(topic + '/touch', RobotiqTouch)
    # Start I2C interface
    tk = rpi.I2CInterface(strips=3, sensors_per_strip=4)
    self.alive = tk.alive()
    num_alive = len(self.alive)
    pressure, temp = tk.get_all_sensors_data()
    cal_matrix = np.array(pressure)
    for j in xrange(99):
      pressure, temp = tk.get_all_sensors_data()
      cal_matrix = np.append(cal_matrix, np.array(pressure))
    cal_matrix.shape = (100, num_alive)
    self.calibration = -np.median(cal_matrix, axis=0)
    old = np.array(pressure)
    # Send sensors data as fast as possible
    touch_msg = RobotiqTouch()
    while not rospy.is_shutdown():
      pressure, temp = tk.get_all_sensors_data()
      for i, value in enumerate(pressure):
        if rpi.nearly_equal(value, 0.0):
          pressure[i] = old[i]
      old = np.array(pressure)
      calibrated = np.array(pressure) + self.calibration
      touch_msg.f0 = calibrated[:step]
      touch_msg.f1 = calibrated[step:2*step]
      touch_msg.f2 = calibrated[2*step:]
      # Publish tactile values within ROS
      touch_pub.publish(touch_msg)
      file_str = StringIO()
      touch_msg.serialize(file_str)
      # Send over udp the tactile values
      self.write_socket.sendto(file_str.getvalue(), (self.write_ip, self.write_port))


if __name__ == '__main__':
  # Set up node
  topic = 'takktile_robotiq'
  rospy.init_node(topic, anonymous=True)
  robotiq = RobotiqUDP(topic)
