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
import cv2
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time

global timing, index_counter, time_start, temp_th1,temp_th2, intercept1,intercept2, flag1,flag2,j1,j2
BUFF_SIZE = 512
P = 0.80


def callback(string):
    global timing, index_counter, time_start,j1,j2, sensorvalue,pressure_zero1,pressure_zero2, pressure_offset1, pressure_offset2
    global IMGcounter, fingertip1, fingertip2, temp_th1, temp_th2,intercept1, intercept2,flag1,flag2, pressure_value1,pressure_value2
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

    fingertip1=np.array([sensorvalue[0,IMGcounter],sensorvalue[1,IMGcounter],sensorvalue[2,IMGcounter],sensorvalue[3,IMGcounter]])
    fingertip2=np.array([sensorvalue[4,IMGcounter],sensorvalue[5,IMGcounter],sensorvalue[6,IMGcounter],sensorvalue[7,IMGcounter]])

    if fingertip1[2] > 2000 or fingertip2[2] > 2000:

        if timing==0: #put flag instead of variable
            pressure_zero1 = fingertip1[0]
            pressure_zero2 = fingertip2[0]
            intercept1 = 165.97
            intercept2 = 188.57
            temp_th1=fingertip1[1]
            temp_th2=fingertip2[1]
            pressure_offset1=((fingertip1[1]+intercept1)/0.1939)-pressure_zero1
            pressure_offset2=((fingertip2[1]+intercept2)/0.2132)-pressure_zero2

        if temp_th1<fingertip1[1] or temp_th1<fingertip1[1]:           #this section is to reduce pressure zero-point float with the temperature.
            if flag1==False:
               intercept1=((0.1939*(pressure_offset1+pressure_zero1))-fingertip1[1])
               flag1=True
            pressure_offset1=((fingertip1[1]+intercept1)/0.1939)-pressure_zero1

        ##########################################

        if temp_th2<fingertip2[1] or temp_th2<fingertip2[1]:
            if flag2==False:
               intercept2=((0.2132*(pressure_offset2+pressure_zero2))-fingertip2[1])
               flag2=True
            pressure_offset2=((fingertip2[1]+intercept2)/0.2132)-pressure_zero2

        ##########################################

        if temp_th1>fingertip1[1]:
            if flag1==True:
               intercept1=((0.1763*(pressure_offset1+pressure_zero1))-fingertip1[1])
               flag1=False
            pressure_offset1=(((fingertip1[1])+intercept1)/0.1763)-pressure_zero1

        if timing==0:
             j1=fingertip1[0]-pressure_offset1
        pressure_value1=fingertip1[0]-pressure_offset1-j1
        fingertip1[0]=(pressure_value1*0.94)+((fingertip1[2]/1000)*0.06)     #adaptive filtering to avoid excessive pressure fluctuations
        if fingertip1[0] > -0.15 and fingertip1[0] < 0.15:
            fingertip1[0]=0
        temp_th1=fingertip1[1]

        ########################################

        if temp_th2>fingertip2[1]:
            if flag2==True:
               intercept2=((0.1800*(pressure_offset2+pressure_zero2))-fingertip2[1])
               flag2=False
            pressure_offset2=(((fingertip2[1])+intercept2)/0.1800) -pressure_zero2
        if timing==0:
            j2=fingertip2[0]-pressure_offset2
        pressure_value2=(fingertip2[0]-pressure_offset2-j2)*1.4
        fingertip2[0]=(pressure_value2*0.94)+((fingertip2[2]/1000)*0.06) ##adaptive filtering to avoid excessive pressure fluctuations
        if fingertip2[0] > -0.15 and fingertip2[0] < 0.15:
            fingertip2[0]=0
        temp_th2=fingertip2[1]
        timing=1

    #### scaling the values to the range 0-255 #####

    fingertip1[0]=(fingertip1[0]/5)*255
    fingertip2[0]=(fingertip2[0]/5)*255

    fingertip1[2]=(fingertip1[2]/2700)*255
    fingertip2[2]=(fingertip2[2]/2700)*255

    fingertip1[3]=(fingertip1[3]/65536)*255
    fingertip2[3]=(fingertip2[3]/65536)*255
    #rospy.loginfo(fingertip1[3])
    
    ###############################################

    if fingertip1[2] < 17 and fingertip2[2] < 17:
        fingertip1[0]=0
        fingertip2[0]=0
        timing=0

    if fingertip1[0]<0 or fingertip2[0]<0 or fingertip1[0]>255 or fingertip2[0]>255:
        fingertip1[0]=0
        fingertip2[0]=0
        timing=0

    IMGcounter = IMGcounter + 1
    if IMGcounter==512:
        IMGcounter=0
    pub0.publish(fingertip1)
    pub1.publish(fingertip2)
    #rospy.loginfo(fingertip1[0])
    #rospy.loginfo(fingertip1[1])









def listener():
    global fingertip1, fingertip2,pub0,pub1
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber("sensors/hand/spx", String, callback, queue_size=10)
            pub0 = rospy.Publisher('sensors/spx_fingertips/0', Floats, queue_size=10)
            pub1 = rospy.Publisher('sensors/spx_fingertips/1', Floats, queue_size=10)
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
    IMGcounter = 0
    pressure_offset1 = 0
    pressure_offset2 = 0
    pressure_zero1=0
    pressure_zero2=0
    a='Value:'
    flag1=False
    flag2=False
    index_counter = 0
    time_start = 0
    alpha=4.6296
    temp_th1=0
    temp_th2=0
    intercept1=165.97
    intercept2=188.57
    j1=0
    j2=0
    plt.ion()
    rospy.init_node('fingertips_SPX_list', anonymous=True)
    listener()
