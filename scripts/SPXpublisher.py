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

global zeroflag, index_counter, time_start, temp_th, intercept, flag, j
BUFF_SIZE = 512
P = 0.80


def callback(string):
    global zeroflag, index_counter, time_start, j, sensorvalue, pressure_zero, pressure_offset, IMGcounter, fingertip, temp_th, intercept, flag, pressure_value,fullscale
    buffer = string.data
    buffer = buffer.split(',')
    if index_counter == 0:
        sensorvalue = np.zeros((12, BUFF_SIZE))
    for i in range(0, 11):
        sensorvalue[i, index_counter] = float(buffer[i])
    if index_counter < BUFF_SIZE - 1:
        index_counter += 1
    else:
        index_counter = 0
        time_start += BUFF_SIZE

    fingertip = np.array([sensorvalue[0, IMGcounter], sensorvalue[1, IMGcounter], sensorvalue[2, IMGcounter], sensorvalue[3, IMGcounter],
                          sensorvalue[4, IMGcounter], sensorvalue[5, IMGcounter], sensorvalue[6, IMGcounter], sensorvalue[7, IMGcounter],
                          sensorvalue[8, IMGcounter], sensorvalue[9, IMGcounter], sensorvalue[10, IMGcounter], sensorvalue[11, IMGcounter]])

    print(fingertip[5])
    if fingertip[2] > 800 or fingertip[6] > 800 or fingertip[10] > 800:

        if zeroflag == 0:  # put flag instead of variable
            intercept[0] = 896.64
            intercept[4] = 885.36
            intercept[8] = 858.34
            intercept[1] = 895.44
            intercept[5] = 884.16
            intercept[9] = 860.27
            fullscale[0] = 0.747518
            fullscale[4] = 3.755431
            fullscale[8] = 4.37586
            fullscale[1] = 30227
            fullscale[5] = 25257
            fullscale[9] = 18905
            m[0]=4.2608
            m[4]=4.6385
            m[8]=5.1346
            m[1]=4.2999
            m[5]=4.6740
            m[9]=5.0898
            for i in ind:
               pressure_zero[i] = fingertip[i]
               temp_th[i] = fingertip[i+1]
               pressure_offset[i] = (m[i]*fingertip[i+1]+intercept[i]) - pressure_zero[i]
               j[i]= fingertip[i] - pressure_offset[i]
        zeroflag = 1
        for i in ind:


            if temp_th[i] < fingertip[i+1] or temp_th[i] == fingertip[i+1]:  # this section is to reduce pressure zero-point float with the temperature.
                if flag[i] == False:
                    intercept[i] = ((pressure_offset[i] + pressure_zero[i])-(m[i] * (fingertip[i+1])))
                    flag[i] = True
                pressure_offset[i] = (m[i]*fingertip[i+1]+intercept[i]) - pressure_zero[i]


            if temp_th[i] > fingertip[i+1]:
                if flag[i] == True:
                    intercept[i+1] = ((pressure_offset[i] + pressure_zero[i])-(m[i+1] * (fingertip[i+1])))
                    flag[i] = False
                pressure_offset[i] = (m[i+1]*fingertip[i+1]+intercept[i+1]) - pressure_zero[i]
            #print("pressure reading", fingertip[0])
            #print("pressure zero", pressure_zero[0])
            #print("pressure offset", pressure_offset[0])
            #print("constantj",j[0])
            #print("intercept", intercept[0])
            pressure_value[i] = fingertip[i] - pressure_offset[i] - j[i]
            temp_th[i]=fingertip[i+1]
            if pressure_value[i]<0:
                pressure_value[i]=0
            fingertip[i] = (pressure_value[i] / fullscale[i]) * 255
            fingertip[i+2] = (fingertip[i+2] / fullscale[i+1]) * 255
            if fingertip[i] < 0:
                fingertip[i] = 0
                zeroflag = 0
            if fingertip [i] > 255:
                fingertip[i]=255
        if fingertip[2] < 15 and fingertip[6] < 15 and fingertip[10] < 15:
            fingertip[0] = 0
            fingertip[4] = 0
            fingertip[8] = 0
            zeroflag = 0

    else:
        fingertip[0] = 0
        fingertip[4] = 0
        fingertip[8] = 0
                # adaptive filtering?
#            if fingertip[i] > -0.15 and fingertip[i] < 0.15:
#                fingertip[i] = 0
#    print("pressure",fingertip[0],",",fingertip[4],",",fingertip[8])
#    print("proximity",fingertip[2],",",fingertip[6],",",fingertip[10])

#    print(pressure_value[0],",",pressure_value[4],",",pressure_value[8])



    IMGcounter = IMGcounter + 1
    if IMGcounter == 512:
        IMGcounter = 0
    pub0.publish(fingertip)

#    print(pressure_value[0])

    # rospy.loginfo(fingertip1[1])


def listener():
    global fingertip, pub0, pub1, pub2
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber("sensors/hand/spx", String, callback, queue_size=10)
            pub0 = rospy.Publisher('sensors/spx_fingertips/raw', Floats, queue_size=10)

            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down Enhanced Grasping!")
        except IOError:
            print("Shuting down Enhanced Grasping!")


if __name__ == '__main__':
    sensorvalue = np.zeros((13, BUFF_SIZE))
    zeroflag = 0
    IMGcounter = 0
    fullscale=np.zeros(10)
    ind=np.array([0,4,8])
    fingertip = np.zeros(12)
    pressure_offset = np.zeros(9)
    intercept = np.zeros(10)
    pressure_zero = np.zeros(9)
    pressure_value = np.zeros(9)
    temp_th = np.zeros(9)
    j = np.zeros(9)
    m = np.zeros(10)
    flag = np.zeros(10,dtype=bool)
    index_counter = 0
    time_start = 0
    plt.ion()
    rospy.init_node('fingertips_SPX_list', anonymous=True)
    listener()



