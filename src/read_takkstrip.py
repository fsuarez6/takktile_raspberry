#! /usr/bin/env python
import smbus, rospy
import time, os

'''
Mimics the arduino code found at: https://github.com/TakkTile/takktile_arduino
'''

class I2CInterface():
  # Constant values
  _attiny_addr = 0xC0 >> 1
  _sensor_all_on = 0x0C >> 1
  _sensor_all_off = 0x0D >> 1
  _max_strips = 8
  _max__sensors = 5
  _wait_time = 2e-3
  # Initialise interface
  def __init__(self, pi_version = 1):
    self._bus = smbus.SMBus(pi_version)
    self._sensors = self._get_addresses()
    self._coefficients = self._get_coefficients(self._sensors)
    while not rospy.is_shutdown():
      print '----------- %d' % (len(self._coefficients.keys()))
      self.start_conversions()
      for sensor, _coefficients in self._coefficients.items():
        print self.get_pressure_temperature(sensor, _coefficients)
      os.system(['clear','cls'][os.name == 'nt'])
  
  def _get_addresses(self):
    addresses = dict()
    for strip in xrange(self._max_strips):
      addresses[strip] = []
      for sensor in xrange(self._max__sensors):
        # Calculate the address
        address = ((strip << 4) + 2 * sensor)  >> 1
        time.sleep(self._wait_time)
        try:
          self._bus.write_quick(address)
          addresses[strip].append(address)
        except:
          continue
    return addresses
  
  def _get_coefficients(self, _sensors):
    coefficients = dict()
    for strip in _sensors.keys():
      for sensor in _sensors[strip]:
        # Select sensor
        time.sleep(self._wait_time)
        self._bus.write_quick(sensor)
        try:
          # Request _coefficients
          block = self._bus.read_i2c_block_data(self._attiny_addr, 0x04)[:8]
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
          # Append the sensor _coefficients
          coefficients[sensor] = short_values
        except:
          rospy.logerr('Failed to get the _coefficients from %d: %s' % (strip, hex(sensor)))
        # Turn off sensor
        self._bus.read_byte(sensor)
    return coefficients
  
  def start_conversions(self):
    # Switch on all the sensors
    self._bus.write_quick(self._sensor_all_on)
    # Start data conversion
    self._bus.write_i2c_block_data(self._attiny_addr, 0x12, [0x01])
    self._bus.read_byte(self._sensor_all_on)
    time.sleep(self._wait_time)
  
  def get_pressure_temperature(self, sensor, _coefficients):
    # Select sensor
    self._bus.write_quick(sensor)
    # Request P/T data
    try:
      block = self._bus.read_i2c_block_data(self._attiny_addr, 0x00)[:4]
    except:
      return (0,0)
    # Turn off sensor
    self._bus.read_byte(sensor)
    # Calculate pressure & temperature
    data = []
    for i in xrange(0, len(block), 2):
      value = (block[i] << 8) | block[i+1]
      if(value >= 2**15):
        value -= 2**16
      data.append(value)
    pressure = float(data[0]) / 64.0
    temp = float(data[1]) / 64.0
    [a0, b1, b2, c12] = _coefficients
    pressure_comp = a0 + (b1 + c12 * temp) * pressure + b2 * temp
    pressure_kpa = ((65.0 / 1023.0) * pressure_comp) + 50.0        # kPa
    temp_celsius = -(temp - 498.0) / 5.35 + 25.0                   # C
    return pressure_comp, temp_celsius
  
if __name__ == '__main__':
  # TODO: Validate inputs
  # pi_version: 0 | 1
  rospy.init_node('read_takkstrip')
  interface = I2CInterface()
