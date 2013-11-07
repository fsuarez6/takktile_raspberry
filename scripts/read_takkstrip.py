#! /usr/bin/env python
import smbus
import time, os
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
  def __init__(self, pi_version = 1):
    self.bus = smbus.SMBus(pi_version)
    self.sensors = self.get_addresses()
    rospy.logdebug('Sensors address: %s' % str(self.sensors))
    self.coefficients = self.get_coefficients(self.sensors)
    while not rospy.is_shutdown():
      print '-----------'
      self.start_conversions()
      for sensor, coefficients in self.coefficients.items():
        print self.get_pressure_temperature(sensor, coefficients)
      os.system(['clear','cls'][os.name == 'nt'])
  
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
      for sensor in sensors[strip]:
        # Select sensor
        self.bus.write_quick(sensor)
        try:
          # Request coefficients
          block = self.bus.read_i2c_block_data(self.attiny_addr, 0x04)[:8]
          short_values = []
          for i in xrange(0, len(block), 2):
            value = (block[i] << 8) | block[i+1]
            if(value >= 2**15):
              value -= 2**16
            short_values.append(float(value))
          short_values[0] /= 8.0
          short_values[1] /= 8192.0
          short_values[2] /= 16384.0
          short_values[3] /= 4194304.0
          # Append the sensor coefficients
          coefficients[sensor] = short_values
        except:
          rospy.logerr('Failed to get the coefficients from %d:%s' % (strip, hex(sensor)))
        # Turn off sensor
        self.bus.read_byte(sensor)
    return coefficients
  
  def start_conversions(self):
    # Turn on all the sensors
    self.bus.write_quick(self.sensor_all_on)
    # Start data conversion
    self.bus.write_i2c_block_data(self.attiny_addr, 0x12, [0x01])
    self.bus.read_byte(self.sensor_all_on)
    time.sleep(2e-3)
  
  def get_pressure_temperature(self, sensor, coefficients):
    # Select sensor
    self.bus.write_quick(sensor)
    # Request P/T data
    try:
      block = self.bus.read_i2c_block_data(self.attiny_addr, 0x00)[:4]
    except:
      return (0,0)
    # Turn off sensor
    self.bus.read_byte(sensor)
    # Calculate pressure & temperature
    data = []
    for i in xrange(0, len(block), 2):
      value = (block[i] << 8) | block[i+1]
      if(value >= 2**15):
        value -= 2**16
      data.append(value)
    pressure = float(data[0]) / 64.0
    temp = float(data[1]) / 64.0
    [a0, b1, b2, c12] = coefficients
    pressure_comp = a0 + (b1 + c12 * temp) * pressure + b2 * temp
    pressure_kpa = ((65.0 / 1023.0) * pressure_comp) + 50.0        # kPa
    temp_celsius = -(temp - 498.0) / 5.35 + 25.0                   # C
    return pressure_comp, temp_celsius
  
if __name__ == '__main__':
  # TODO: Validate inputs
  # pi_version: 0 | 1
  rospy.init_node('read_takkstrip')
  interface = I2CInterface()
