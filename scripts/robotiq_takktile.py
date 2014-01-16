#! /usr/bin/env python
import rospy, math
import numpy as np
# Sockets
import socket, struct
from cStringIO import StringIO
# TakkStrip
from takktile_raspberry import I2CInterface
# Messages
from takktile_raspberry.msg import RobotiqTouch

def nearly_equal(a,b,sig_fig=3):
  return (a==b or int(a*10**sig_fig) == int(b*10**sig_fig))

class RobotiqTakktile():
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
    tk = I2CInterface()
    self.alive = tk.alive()
    num_alive = len(self.alive)
    pressure, temp = tk.get_all_sensors_data()
    self.calibration = -np.array(pressure)
    old = np.array([0]*num_alive)
    # Send sensors data as fast as possible
    touch_msg = RobotiqTouch()
    while not rospy.is_shutdown():
      pressure, temp = tk.get_all_sensors_data()
      calibrated = np.array(pressure) + self.calibration
      for i, value in enumerate(calibrated):
        if nearly_equal(value, 0.0):
          calibrated[i] = old[i]
      touch_msg.f0 = calibrated[:step]
      touch_msg.f1 = calibrated[step:2*step]
      touch_msg.f2 = calibrated[2*step:]
      #~ touch_pub.publish(touch_msg)
      file_str = StringIO()
      touch_msg.serialize(file_str)
      self.write_socket.sendto(file_str.getvalue(), (self.write_ip, self.write_port))
      old = np.array(calibrated)


if __name__ == '__main__':
  # Set up node
  topic = 'robotiq_takktile'
  rospy.init_node(topic, anonymous=True)
  robotiq = RobotiqTakktile(topic)
