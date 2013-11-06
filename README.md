TakkStrip with Raspberry Pi
==================

## Setup

These instructions were obtain from this [instructable](http://www.instructables.com/id/Raspberry-Pi-I2C-Python/).

### Enable I2C


On the Pi, I2C is disabled by default. The first thing to do, is run this command: 
`$ sudo nano /etc/modprobe.d/raspi-blacklist.conf` 

In this file, there is a comment, and two lines. Add a hash before the I2C line, to comment it out.

Original:
```
# blacklist spi and i2c by default (many users don't need them)

blacklist spi-bcm2708
blacklist i2c-bcm2708
``` 

Convert to this:
```
# blacklist spi and i2c by default (many users don't need them)

blacklist spi-bcm2708
#blacklist i2c-bcm2708
``` 

### Enable kernel I2C Module

The next thing to do is add the I2C module to the kernel. Run:
`$ sudo nano /etc/modules`

You should see the following file:
```
# /etc/modules: kernel modules to load at boot time.
#
# This file contains the names of kernel modules that should be loaded
# at boot time, one per line. Lines beginning with "#" are ignored.
# Parameters can be specified after the module name.

snd-bcm2835
``` 
This should have the line i2c-dev added to the end.

Final file:
```
# /etc/modules: kernel modules to load at boot time.
#
# This file contains the names of kernel modules that should be loaded
# at boot time, one per line. Lines beginning with "#" are ignored.
# Parameters can be specified after the module name.

snd-bcm2835
i2c-dev
``` 
### Install Necessary Packages

`$ sudo apt-get install i2c-tools python-smbus`

### Allow Pi User to Access I2C Devices

`$ sudo adduser pi i2c`

Now reboot:
`$ sudo reboot`

### Testing the Installation

After that you should see the i2c devices:
```
$ ls /dev/i2c*
/dev/i2c-0	/dev/i2c-1
``` 
Simple test to scan the i2c bus:
```
$ i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
``` 
