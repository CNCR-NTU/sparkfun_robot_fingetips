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
__description__ = 'Subscribe SPX Fingertips sensors'
__compatibility__ = "Python 2 and Python 3"
__platforms__ = "Sawyer and AR10 hand"
import rospy
from rospy_tutorials.msg import Floats
from std_msgs.msg import String
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time
global timing, pressure, sensorvalue, temperature, light, proximity, pressure_offset1, pressure_offset2




def callback(string):
    global timing, sensorvalue, pressure, temperature, light, proximity, pressure_offset1, pressure_offset2

    buffer = string.data
    buffer=buffer.split(',')
    sensorvalue = np.asarray(buffer)
    if timing==0:
        pressure_offset1=float(sensorvalue[0])
        pressure_offset2=float(sensorvalue[4])
    pressure=np.asarray([float(sensorvalue[0])-pressure_offset1,float(sensorvalue[4])-pressure_offset2])
    temperature=np.asarray([float(sensorvalue[1]),float(sensorvalue[5])])
    proximity=np.asarray([float(sensorvalue[2]),float(sensorvalue[6])])
    light=np.asarray([float(sensorvalue[3]),float(sensorvalue[7])])
    timing+=counter


def main():
    global pressure, temperature, light, proximity

    while not rospy.is_shutdown():
        fig1=plt.figure("sensor1")
        fig1.subplots_adjust(wspace=.5)
        ax1=plt.subplot(221)
        plt.title("Pressure1")
        plt.ylabel("Pressure [hPa]")
        plt.xlabel("time [s]")
        ax1.set_xlim([timing-200, timing+100])
        plt.plot(timing,pressure[0],'bo', label='pressure1')

        ax2=plt.subplot(222)
        plt.title("Temperature1")
        plt.ylabel("Temperature [C]")
        plt.xlabel("time [s]")
        plt.plot(timing,temperature[0],'bo', label='temperature1')
        ax2.set_xlim([timing-200, timing+200])

        ax3=plt.subplot(223)
        plt.title("Proximity1")
        plt.ylabel("Proximity")
        plt.xlabel("time [s]")
        plt.plot(timing,proximity[0],'bo', label='proximity1')
        ax3.set_xlim([timing-200, timing+200])

        ax4=plt.subplot(224)
        plt.title("Light1")
        plt.ylabel("Light [lux]")
        plt.xlabel("time [s]")
        plt.plot(timing,light[0],'bo', label='light1')
        ax4.set_xlim([timing-200, timing+200])

        fig2=plt.figure("sensor2")
        fig2.subplots_adjust(wspace=.5)
        ax1=plt.subplot(221)
        plt.title("Pressure2")
        plt.ylabel("Pressure [hPa]")
        plt.xlabel("time [s]")
        plt.plot(timing,pressure[1],'bo', label='pressure2')
        ax1.set_xlim([timing-200, timing+200])

        ax2=plt.subplot(222)
        plt.title("Temperature2")
        plt.ylabel("Temperature [C]")
        plt.xlabel("time [s]")
        plt.plot(timing,temperature[1],'bo', label='temperature2')
        ax2.set_xlim([timing-200, timing+200])

        ax3=plt.subplot(223)
        plt.title("Proximity2")
        plt.ylabel("Proximity")
        plt.xlabel("time [s]")
        plt.plot(timing,proximity[1],'bo', label='proximity2')
        ax3.set_xlim([timing-200, timing+200])

        ax4=plt.subplot(224)
        plt.title("Light2")
        plt.ylabel("Light [lux]")
        plt.xlabel("time [s]")
        plt.plot(timing,light[1],'bo', label='light2')
        ax4.set_xlim([timing-200, timing+200])
        
        plt.pause(0.001)

def listener()
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
    sensorvalue=[0,0,0,0,0,0,0,0]
    pressure=[0,0]
    temperature = [0, 0]
    light=[0,0]
    proximity=[0,0]
    timing=0
    counter=1
    pressure_offset1 = 0
    pressure_offset2 = 0
    rospy.init_node('fingertips_SPX_list', anonymous=True)
    listener()

