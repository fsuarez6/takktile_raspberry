#! /usr/bin/env python
import smbus
import time
import rospy

'''
Mimics the arduino code found at: https://github.com/TakkTile/takktile_arduino
'''

class I2CInterface():
  # Constant values
  attiny_addr = 0xC0 >> 1
  sensor_all_on = 0x0C >> 1
  sensor_all_off = 0x0D >> 1
  max_strips = 8
  max_sensors = 5
  # Initialise interface
  def __init__(self, pi_version = 1, address_type='old'):
    self.bus = smbus.SMBus(pi_version)
    self.sensors_addr = self.get_addresses()
    rospy.logdebug('Sensors address: %s' % str(self.sensors_addr))
    self.get_coefficients(self.sensors_addr)
  
  def get_addresses(self):
    addresses = dict()
    for strip in xrange(self.max_strips):
      addresses[strip] = []
      for sensor in xrange(self.max_sensors):
        # Calculate the address
        address = ((strip << 4) + 2 * sensor)  >> 1
        try:
          self.bus.write_quick(address)
          addresses[strip].append(address)
        except:
          continue
    return addresses
  
  def get_coefficients(self, sensors):
    coefficients = dict()
    for strip in sensors.keys():
      coefficients[strip] = []
      for sensor in sensors[strip]:
        # Select sensor
        self.bus.write_quick(sensor)
        try:
          # Request coefficients
          block = self.bus.read_i2c_block_data(self.attiny_addr, 0x04)[:8]
          data = []
          data.append((block[0] << 8) | block[1])
          data.append((block[2] << 8) | block[3]) 
          data.append((block[4] << 8) | block[5])
          data.append((block[6] << 8) | (block[7] >> 2))
          short_values = []
          for value in data:
            if(value >= 2**15):
              value -= 2**16
            short_values.append(float(value))
          short_values[0] /= 8.0
          short_values[1] /= 8192.0
          short_values[2] /= 16384.0
          short_values[3] /= 4194304.0
          # Append the sensor coefficients
          coefficients[strip].append(short_values)
        except:
          rospy.logerr('Failed to get the coefficients from %d:%s' % (strip, hex(sensor)))
        # Turn off sensor
        self.bus.read_byte(sensor)
    return coefficients
  
if __name__ == '__main__':
  # TODO: Validate inputs
  # pi_version: 0 | 1
  rospy.init_node('read_takkstrip')
  interface = I2CInterface()
