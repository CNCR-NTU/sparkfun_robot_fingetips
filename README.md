# sparkfun_robot_fingertips

This is repository for hosting the Sparkfun Robot Fingertips sensors sofware released under the lgpl3.0 license.

# Requirements

## Documentation
* [LPS25HB user manual](https://github.com/CNCR-NTU/sparkfun_robot_fingetips/blob/master/doc/LPS25HB.pdf)
* [VCNL4040 user manual](https://github.com/CNCR-NTU/sparkfun_robot_fingetips/blob/master/doc/vcnl4040.pdf)

## Hardware
* 3x [Sparkfun Robotic Fingertips](https://www.sparkfun.com/products/14687)
* Arduino Board
* USB cable for connecting Arduino board
* Host pc

## Software
* Ubuntu Linux 18.04 LTS
* ROS Melodic [installed](http://wiki.ros.org/melodic/Installation/Ubuntu) and [configured](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
* Arduino IDE
* Softwarewire Arduino library

# Installation procedure:
## Step 1: clone the repository
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/CNCR-NTU/sparkfun_robot_fingertips.git
```
## Step 2: compile and install
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.sh
```

# Understanding the data
The data is retreived from the arduino board through the ROS "serial" library. The data coming from the three sensors is published as a string with format:

"pressure1,temp1,proximity1,light1,pressure2,temp2,proximity2,light2,pressure3,temp3,proximity3,light3"

The output vector (sensorValues) of the subscriber is an array containig the numerical values of the sensor data, with format:

string[pressure1,temp1,proximity1,light1,pressure2,temp2,proximity2,light2,pressure3,temp3,proximity3,light3]



# Contacts
Computational Neurosciences and Cognitive Robotics Group at the Nottingham Trent University.

Gabriele Gaudino <gabriele.gaudino2018@my.ntu.ac.uk>

Pedro Machado <pedro.baptistamachado@ntu.ac.uk>

Martin McGinnity <martin.mcginnity@ntu.ac.uk>
