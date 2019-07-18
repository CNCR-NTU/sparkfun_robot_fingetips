#!/usr/bin/env python
from __future__ import print_function

# -*- coding: utf-8 -*-
""" 
:ABSTRACT: 
This script is part of the of the enhanced grasp project 
:REQUIRES: 
: 
:AUTHOR:  Gabriele Gaudino 
:ORGANIZATION: Nottingham Trent University 
:CONTACT: n0859404@my.ntu.ac.uk 
:SINCE: 5/06/2019 
:VERSION: 0.1 
This file is part of <project> project. 
the Robot 2 Robot interaction project can not be copied and/or distributed without the express 
permission of Prof. Martin McGinnity <martin.mcginnity@ntu.ac.uk> 
Copyright (C) 2019 All rights reserved, Nottingham Trent University 
Computational Neuroscience and Cognitive Robotics Laboratory 
email:  pedro.baptistamachado@ntu.ac.uk 
website: https://www.ntu.ac.uk/research/groups-and-centres/groups/computational-neuroscience-and-cognitive-robotics-laboratory 
"""
# ===============================================================================
# PROGRAM METADATA
# ===============================================================================
__author__ = 'Gabriele Gaudino'
__contact__ = 'n0859404@my.ntu.ac.uk'
__copyright__ = 'Enhanced grasping project can not be copied and/or distributed \
without the express permission of Prof. Martin McGinnity <martin.mcginnity@ntu.ac.uk'
__license__ = '2019 (C) CNCR@NTU, All rights reserved'
__date__ = '13/02/2019'
__version__ = '0.1'
__file_name__ = 'SPXvisualize.py'
__description__ = 'Subscribe SPX Fingertips sensrs'
__compatibility__ = "Python 2 and Python 3"
__platforms__ = "Sawyer and AR10 hand"
import rospy
from rospy_tutorials.msg import Floats
from std_msgs.msg import String
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time

global timing, index_counter, time_start, pressure_offset1, pressure_offset2
BUFF_SIZE = 512
P = 0.80


def callback(string):
    global timing, index_counter, time_start, sensorvalue, pressure_offset1, pressure_offset2
    buffer = string.data
    buffer = buffer.split(',')
    if index_counter == 0:
        sensorvalue = np.zeros((8, BUFF_SIZE))
    for i in range(0, 8):
        sensorvalue[i, index_counter] = float(buffer[i])
    if index_counter < BUFF_SIZE - 1:
        index_counter += 1
    else:
        index_counter = 0
        time_start += BUFF_SIZE
    if timing == 0:
        pressure_offset1[0] = float(sensorvalue[0, 0]) - 2
        pressure_offset1[1] = float(sensorvalue[0, 0]) + 15
        pressure_offset2[0] = float(sensorvalue[4, 0]) - 2
        pressure_offset2[1] = float(sensorvalue[4, 0]) + 15
        timing = 1

        # sensorvalue[0]= sensorvalue[0]-pressure_offset1


# sensorvalue[4]= sensorvalue[4]-pressure_offset2
# temperature=np.asarray([float(sensorvalue[1]),float(sensorvalue[5])])
# proximity=np.asarray([float(sensorvalue[2]),float(sensorvalue[6])])
# light=np.asarray([float(sensorvalue[3]),float(sensorvalue[7])])
# timing+=1


def main():
    global sensorvalue, time_start, pressure_offset1, pressure_offset2
    while not rospy.is_shutdown():
        timing = np.asarray(range(time_start, time_start + BUFF_SIZE))
        timing = np.dot(timing, 1. / 8.)
        plt.figure("sensor1")
        ax1 = plt.subplot(711)
        plt.title("Pressure1")
        plt.ylabel("Pressure \n[hPa]")
        ax1.set_ylim([pressure_offset1[0], pressure_offset1[1]])
        plt.plot(timing, sensorvalue[0], c='k', ls='-', label='pressure1')

        ax2 = plt.subplot(713)
        plt.title("Temperature1")
        plt.ylabel("Temperature \n[C]")
        ax2.set_ylim([0, 45])
        plt.plot(timing, sensorvalue[1], c='k', ls='-', label='temperature1')

        plt.subplot(715)
        plt.title("Proximity1")
        plt.ylabel("Proximity")
        plt.plot(timing, sensorvalue[2], c='k', ls='-', label='proximity1')

        plt.subplot(717)
        plt.title("Light1")
        plt.ylabel("Light [lux]")
        plt.plot(timing, sensorvalue[3], c='k', ls='-', label='light1')
        plt.pause(0.001)

        plt.figure("sensor2")
        ax3 = plt.subplot(711)
        plt.title("Pressure2")
        plt.ylabel("Pressure [hPa]")
        plt.xlabel("time [s]")
        ax3.set_ylim([pressure_offset2[0], pressure_offset2[1]])
        plt.plot(timing, sensorvalue[4], c='b', ls='-', label='pressure2')

        ax4 = plt.subplot(713)
        plt.title("Temperature2")
        plt.ylabel("Temperature [C]")
        plt.xlabel("time [s]")
        ax4.set_ylim([0, 45])
        plt.plot(timing, sensorvalue[5], c='b', ls='-', label='temperature2')

        plt.subplot(715)
        plt.title("Proximity2")
        plt.ylabel("Proximity")
        plt.xlabel("time [s]")
        plt.plot(timing, sensorvalue[6], c='b', ls='-', label='proximity2')

        plt.subplot(717)
        plt.title("Proximity2")
        plt.ylabel("Proximity")
        plt.xlabel("time [s]")
        plt.plot(timing, sensorvalue[6], c='b', ls='-', label='proximity2')
        plt.clf()

        plt.pause(0.001)
        plt.show()


def listener():
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber("sensors/hand/spx", String, callback, queue_size=10)
            main()
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down Enhanced Grasping!")
        except IOError:
            print("Shuting down Enhanced Grasping!")


if __name__ == '__main__':
    sensorvalue = np.zeros((8, BUFF_SIZE))
    timing = 0
    counter = 1
    pressure_offset1 = [0, 0]
    pressure_offset2 = [0, 0]

    index_counter = 0
    time_start = 0
    plt.ion()

    rospy.init_node('fingertips_SPX_list', anonymous=True)
    listener()
