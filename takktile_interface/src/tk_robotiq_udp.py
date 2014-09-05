#! /usr/bin/env python
import takktile_interface as rpi
import math, argparse, signal
import numpy as np
# Sockets
import socket, struct
from cStringIO import StringIO

interrupted = False

def signal_handler(signum, frame):
    global interrupted
    interrupted = True

if __name__ == '__main__':
  # Parse the arguments
  parser = argparse.ArgumentParser(description='Read the takktile sensor from multiple arrays')
  parser.add_argument('--ip_dst', dest='ip_dst', type=str, default='127.0.0.1',
                        help='IP address where the UDP message will be sent')
  parser.add_argument('--port_dst', dest='port_dst', type=int, default=5050,
                        help='UDP port used for sending the readings')
  args = parser.parse_args()
  # Handle the signal interruption (Ctrl+C)
  signal.signal(signal.SIGINT, signal_handler)
  # Set up sender socket
  write_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  write_ip = args.ip_dst
  write_port = args.port_dst
  print 'UDP Socket sending to [udp://%s:%d]' % (write_ip, write_port)
  # Start I2C interface
  strips=3
  sensors_per_strip=4
  tk = rpi.I2CInterface(strips, sensors_per_strip)
  alive = tk.alive()
  num_alive = len(alive)
  pressure, temp = tk.get_all_sensors_data()
  cal_matrix = np.array(pressure)
  for j in xrange(99):
    pressure, temp = tk.get_all_sensors_data()
    cal_matrix = np.append(cal_matrix, np.array(pressure))
  cal_matrix.shape = (100, num_alive)
  calibration = -np.median(cal_matrix, axis=0)
  old = np.array(pressure)
  # Send sensors data as fast as possible
  while True:
    pressure, temp = tk.get_all_sensors_data()
    for i, value in enumerate(pressure):
      if rpi.nearly_equal(value, 0.0):
        pressure[i] = old[i]
    old = np.array(pressure)
    calibrated = np.array(pressure) + calibration
    f0 = calibrated[:sensors_per_strip]
    f1 = calibrated[sensors_per_strip:2*sensors_per_strip]
    f2 = calibrated[2*sensors_per_strip:]
    # Serialize UDP message
    buff = StringIO()
    buff.write(struct.pack('<I', sensors_per_strip))
    pattern = '<%sf'%sensors_per_strip
    buff.write(struct.pack(pattern, *f0))
    buff.write(struct.pack('<I', sensors_per_strip))
    pattern = '<%sf'%sensors_per_strip
    buff.write(struct.pack(pattern, *f1))
    buff.write(struct.pack('<I', sensors_per_strip))
    pattern = '<%sf'%sensors_per_strip
    buff.write(struct.pack(pattern, *f2))
    # Send over udp the tactile values
    write_socket.sendto(buff.getvalue(), (write_ip, write_port))
    if interrupted:
      print 'Ctrl+C received, exiting...'
      break
