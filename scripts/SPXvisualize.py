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
import os
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time

global timing, index_counter, time_start, pressure_offset1, pressure_offset2
BUFF_SIZE = 512
P = 0.80


def callback(string):
    global timing, index_counter, time_start, sensorvalue, pressure_offset1, pressure_offset2, IMGcounter, fingertip1, fingertip2
    buffer = string.data
    buffer = buffer.split(',')
    if index_counter == 0:
        sensorvalue = np.zeros((8, BUFF_SIZE))
    for i in range(0, 8):
        sensorvalue[i, index_counter] = float(buffer[i])

    if index_counter < BUFF_SIZE - 1:
        index_counter += 1

      #  rospy.loginfo((sensorvalue[0, counter]))

    else:
        index_counter = 0
        time_start += BUFF_SIZE
    if timing == 0:
        pressure_offset1 = float(sensorvalue[0, 0])
        pressure_offset2 = float(sensorvalue[4, 0])
        timing = 1
    fingertip1=np.array([sensorvalue[0,IMGcounter],sensorvalue[1,IMGcounter],sensorvalue[2,IMGcounter],sensorvalue[3,IMGcounter]])
    fingertip2=np.array([sensorvalue[4,IMGcounter],sensorvalue[5,IMGcounter],sensorvalue[6,IMGcounter],sensorvalue[7,IMGcounter]])
   # rospy.loginfo(sensorvalue[0, IMGcounter])
    IMGcounter = IMGcounter + 1
    if IMGcounter==512:
        IMGcounter=0








def main():
    global sensorvalue, time_start, pressure_offset1, pressure_offset2, IMGcounter, fingertip1, fingertip2,pub0,pub1
    while not rospy.is_shutdown():
        timing = np.asarray(range(time_start, time_start + BUFF_SIZE))
        timing = np.dot(timing, 1. / 8.)
        while not rospy.is_shutdown():

            fig1 = plt.figure("sensor1")
            fig1.subplots_adjust(wspace=.5)

            ax1 = plt.subplot(421)
            plt.title("SENSOR 1")
            plt.ylabel("PRESSURE \n [hPa]")
            plt.xlabel("time [s]")
            plt.bar(0, fingertip1[0]- pressure_offset1)
            ax1.set_ylim(0, 10)
            ax1.text(-0.1, 1, a + str(round(fingertip1[0],2)))

            ax2 = plt.subplot(423)
            plt.ylabel("TEMPERATURE \n[C]")
            ax2.set_ylim([0, 45])
            plt.bar(0, fingertip1[1])
            ax2.text(-0.1, 1, a + str(round(fingertip1[1],2)))

            ax3 = plt.subplot(425)
            plt.ylabel("PROXIMITY")
            ax3.set_ylim(0, 10000)
            plt.bar(0, fingertip1[2])
            ax3.text(-0.1, 1, a + str(round(fingertip1[2],2)))

            ax4 = plt.subplot(427)
            plt.ylabel("LIGHT \n [lux]")
            ax4.set_ylim(0, 1000)
            plt.bar(0, fingertip1[3])
            ax4.text(-0.1, 1, a + str(round(fingertip1[3],2)))

            ax5 = plt.subplot(422)
            plt.title("SENSOR 2")
            plt.bar(0, fingertip2[0]-pressure_offset2, color='cyan')
            ax5.set_ylim(0, 10)
            ax5.text(-0.1, 1, a + str(round(fingertip2[0],2)))

            ax6 = plt.subplot(424)
            ax6.set_ylim([0, 45])
            plt.bar(0, fingertip2[1], color='cyan')
            ax6.text(-0.1, 1, a + str(round(fingertip2[1],2)))

            ax7 = plt.subplot(426)
            ax7.set_ylim(0, 10000)
            plt.bar(0, fingertip2[2], color='cyan')
            ax7.text(-0.1, 1, a + str(round(fingertip2[2],2)))

            ax8 = plt.subplot(428)
            ax8.set_ylim(0, 1000)
            plt.bar(0, fingertip2[3], color='cyan')
            ax8.text(-0.1, 1, a + str(round(fingertip2[3],2)))

            plt.show()
            plt.pause(0.001)
            plt.clf()
            rospy.loginfo(fingertip1)
            pub0.publish(fingertip1)
            pub1.publish(fingertip2)






def listener():
    global fingertip1, fingertip2,pub0,pub1
    while not rospy.is_shutdown():
        try:

            rospy.Subscriber("sensors/hand/spx", String, callback, queue_size=10)
            pub0 = rospy.Publisher('sensors/spx_fingertips/0', Floats, queue_size=10)
            pub1 = rospy.Publisher('sensors/spx_fingertips/1', Floats, queue_size=10)
            main()
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down Enhanced Grasping!")
        except IOError:
            print("Shuting down Enhanced Grasping!")


if __name__ == '__main__':
    sensorvalue = np.zeros((8, BUFF_SIZE))
    fingertip1=np.zeros(4)
    fingertip2=np.zeros(4)
    timing = 0
    IMGcounter = -1
    pressure_offset1 = 0
    pressure_offset2 = 0
    a='Value:'
    index_counter = 0
    time_start = 0


    plt.ion()

    rospy.init_node('fingertips_SPX_list', anonymous=True)
    listener()
