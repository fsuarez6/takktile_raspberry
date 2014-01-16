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

[![Build Status](https://travis-ci.org/fsuarez6/takktile_raspberry.png?branch=groovy-devel)](https://travis-ci.org/fsuarez6/takktile_raspberry)

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
wstool init .
wstool merge https://raw.github.com/fsuarez6/takktile_raspberry/groovy-devel/takktile_raspberry.rosinstall
wstool update
``` 
Install any missing dependencies using rosdep:
```
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro groovy
``` 
Now compile your ROS workspace. e.g.
```
cd ~/catkin_ws && catkin_make
``` 

### Testing Installation

Be sure to always source the appropriate ROS setup file, which for Hydro is done like so:
```
source /opt/ros/hydro/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try the `.launch` files in the `takktile_raspberry` package:
```
roslaunch takktile_raspberry robotiq_takktile.launch
``` 

## Changelog

### 0.1.0 (2014-01-16)
* Initial Release

## Roadmap
TODO

[![Bitdeli Badge](https://d2weczhvl823v0.cloudfront.net/fsuarez6/takktile_raspberry/trend.png)](https://bitdeli.com/free "Bitdeli Badge")

