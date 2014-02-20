TakkStrip with Raspberry Pi
==================

ROS packages developed by the [Group of Robots and Intelligent Machines](http://www.romin.upm.es/) from the [Universidad Politécnica de Madrid](http://www.upm.es/internacional). This group is part of the [Centre for Automation and Robotics](http://www.car.upm-csic.es/) (CAR UPM-CSIC). On going development continues in the groovy-devel branch.

**Maintainer:** Francisco Suárez Ruiz, [http://www.romin.upm.es/fsuarez/](http://www.romin.upm.es/fsuarez/)

### Documentation

  * See the installation instructions below.
  * This repository.
  * Throughout the various files in the packages.
  * For questions, please use [http://answers.ros.org](http://answers.ros.org)

### Build Status

[![Build Status](https://travis-ci.org/fsuarez6/takktile_ros.png?branch=groovy-devel)](https://travis-ci.org/fsuarez6/takktile_ros)

## Raspberry Setup

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
$ i2cdetect -y -a 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
``` 

## ROS Packages Installation

### Basic Requirements

  1. Install [ROS Groovy](http://wiki.ros.org/groovy/Installation/Raspbian) on Raspberry PI/Raspbian (**Installing binary packages** Recommended)

### Repository Installation

Go to your ROS working directory. e.g.
```
cd ~/catkin_ws/src
``` 
Use the `wstool` to install the repository
```
$ sudo apt-get install python-wstool
$ wstool init .
$ wstool merge https://raw.github.com/fsuarez6/takktile_ros/groovy-devel/takktile_ros.rosinstall
$ wstool update
``` 
Install any missing dependencies using rosdep:
```
$ rosdep update
$ rosdep install --from-paths . --ignore-src --rosdistro groovy
``` 
Now compile your ROS workspace. e.g.
```
$ cd ~/catkin_ws && catkin_make
``` 

### Testing Installation (With ROS)

Be sure to always source the appropriate ROS setup file, which for Groovy is done like so:
```
$ source /opt/ros/groovy/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try the `.launch` file in the `takktile_raspberry` package:
```
$ roslaunch takktile_raspberry takktile_robotiq.launch ip_dst:=127.0.0.1 port_dst:=5050
``` 

You should be able to echo the tactile values within ROS running in OTHER console:
```
$ rostopic echo /takktile_robotiq/touch
---
f0: [1.404296875, -1.513671875, -0.20703125, 0.234375]
f1: [3.0390625, 0.0, -0.0518798828125, 0.9111328125]
f2: [0.91851806640625, 0.0, 0.823822021484375, 0.6637859344482422]
---
``` 

Be patient, the Raspberry Pi may take a while

### Testing Installation (Without ROS)

If you don't want to use ROS you can use the `read_takkstrip` python module directly. Go to the folder `/takktile_ros/takktile_raspberry/src/takktile_raspberry` and run the following command:

```
$ python read_takkstrip.py --strips 3 --sensors_per_strip 4
(Pressure [KPa], Temperature [C])
Sensor 0x0: (347.248047, 24.439252)
Sensor 0x1: (234.890625, 24.252336)
Sensor 0x2: (266.414062, 24.626168)
Sensor 0x3: (248.351562, 24.439252)
Sensor 0x8: (363.914062, 24.626168)
Sensor 0x9: (421.414062, 24.626168)
Sensor 0xa: (489.779663, 25.186916)
Sensor 0xb: (423.477295, 24.252336)
Sensor 0x10: (447.142822, 25.000000)
Sensor 0x11: (455.427246, 24.252336)
Sensor 0x12: (507.557922, 25.186916)
Sensor 0x13: (1201.648240, 24.065421)
``` 

## Changelog

### 0.1.0 (2014-02-20)
* Initial Release

## Roadmap
TODO

[![Bitdeli Badge](https://d2weczhvl823v0.cloudfront.net/fsuarez6/takktile_ros/trend.png)](https://bitdeli.com/free "Bitdeli Badge")

