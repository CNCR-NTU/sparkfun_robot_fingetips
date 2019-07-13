# sparkfun_robot_fingetips

This is repository for hosting the Sparkfun Robot Fingertips sensors sofware released under the lgpl3.0 license.

# Requirements

## Documentation
* TBC

## Hardware
* 3x [Sparkfun Robotic Fingertips](https://www.sparkfun.com/products/14687)
* TBC
* Host pc

## Software
* Ubuntu Linux 18.04 LTS
* ROS Melodic [installed](http://wiki.ros.org/melodic/Installation/Ubuntu) and [configured](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

# Installation procedure:
## Step 1: Connect the equipment 
Figure 1 shows the configuration setup

![](https://github.com/pedrombmachado/biotac_sp/blob/master/doc/Biotac.png)

Figure 1: Biotac setup
  
## Step 2: Update the OS and install base packets

```
$ sudo apt update & sudo apt upgrade -y & sudo apt dist-upgrade -y & sudo apt autoremove -y & sudo apt autoclean -y
$ sudo apt install build-essential git terminator
```

## Step 3: clone the repository
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/CNCR-NTU/biotac_sp_ros.git
```

## Step 4: install the drivers
```
$ cd biotac_sp_ros
$ ./installCheetahDriver.sh
```

## Step 5: compile and install
```
$ cd ~/catkin_ws
$ catkin_make
$ catkin_make install
$ source devel/setup.sh
```

## Step 6: run

Run terminator

`$ roslaunch biotac.launch`


# Understanding the data
TBC
# Contacts
Computational Neurosciences and Cognitive Robotics Group at the Nottingham Trent University.

Gabriele Gaudino <gabriele.gaudino2018@my.ntu.ac.uk>

Pedro Machado <pedro.baptistamachado@ntu.ac.uk>

Martin McGinnity <martin.mcginnity@ntu.ac.uk>
