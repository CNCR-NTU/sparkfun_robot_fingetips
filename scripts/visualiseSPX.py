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
__author__ = 'Gabriele Gaudino, Pedro Machado'
__contact__ = 'n0859404@my.ntu.ac.uk, pedro.baptistamachado@ntu.ac.uk'
__copyright__ = 'Enhanced grasping project can not be copied and/or distributed \
without the express permission of Prof. Martin McGinnity martin.mcginnity@ntu.ac.uk'
__license__ = '2019 (C) CNCR@NTU, All rights reserved'
__date__ = '13/02/2019'
__version__ = '0.1'
__file_name__ = 'visualiseSPX.py'
__description__ = 'Visualise SPX Fingertips sensors'
__compatibility__ = "Python 2 and Python 3"
__platforms__ = "Sawyer and AR10 hand"
import rospy
import os
import cv2
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time

global zeroflag, index_counter, time_start, temp_th, intercept, flag, j
BUFF_SIZE = 512
global matrix, mat_index
matrix=np.zeros([BUFF_SIZE,4,3])
P = 0.80
visualisationFlag=True
scale_percent=6000



def callback(data):
    global zeroflag1,zeroflag2,zeroflag3, index_counter, time_start, j, sensorvalue, pressure_zero, pressure_offset, IMGcounter, fingertip, temp_th, intercept, flag, pressure_value,pressure_previous, fullscale
    buffer = data.data
    if index_counter == 0:
        sensorvalue = np.zeros((12, BUFF_SIZE))
    for i in range(0, 11):
        sensorvalue[i, index_counter] = buffer[i]
    if index_counter < BUFF_SIZE - 1:
        index_counter += 1
    else:
        index_counter = 0
        time_start += BUFF_SIZE

    fingertip = np.asarray(([sensorvalue[0, IMGcounter], sensorvalue[1, IMGcounter], sensorvalue[2, IMGcounter], sensorvalue[3, IMGcounter],
                          sensorvalue[4, IMGcounter], sensorvalue[5, IMGcounter], sensorvalue[6, IMGcounter], sensorvalue[7, IMGcounter],
                          sensorvalue[8, IMGcounter], sensorvalue[9, IMGcounter], sensorvalue[10, IMGcounter], sensorvalue[11, IMGcounter]]),dtype=np.float32)
    if fingertip[2] > 50 or fingertip[6] > 50 or fingertip[10] > 50:
        if zeroflag1 == 0:
            intercept[0] = 896.64
            intercept[1] = 895.44
            fullscale[0] = 0.27
            fullscale[1] = 45
            fullscale[2] = 3022
            fullscale[3] = 65500
            m[0] = 4.2608
            m[1] = 4.2999
            zeroflag1 = 1
            pressure_zero[0] = fingertip[1]
            temp_th[0] = fingertip[1]
            pressure_offset[0] = (m[0] * fingertip[1] + intercept[0]) - pressure_zero[0]
            j[0] = fingertip[0] - pressure_offset[0]
        if zeroflag2 == 0:
            intercept[4] = 885.36
            intercept[5] = 884.16
            fullscale[4] = 1.3
            fullscale[5] = 45
            fullscale[6] = 2500
            fullscale[7] = 65500
            m[4] = 4.6385
            m[5] = 4.6740
            zeroflag2 = 1
            pressure_zero[4] = fingertip[4]
            temp_th[4] = fingertip[5]
            pressure_offset[4] = (m[4] * fingertip[5] + intercept[4]) - pressure_zero[4]
            j[4] = fingertip[4] - pressure_offset[4]
        if zeroflag3 == 0:
            intercept[8] = 858.34
            intercept[9] = 860.27
            fullscale[8] = 1.6
            fullscale[9] = 45
            fullscale[10] = 1890
            fullscale[11] = 65500
            m[8] = 5.1346
            m[9] = 5.0898
            zeroflag3 = 1
            pressure_zero[8] = fingertip[8]
            temp_th[8] = fingertip[9]
            pressure_offset[8] = (m[8] * fingertip[9] + intercept[8]) - pressure_zero[8]
            j[8] = fingertip[8] - pressure_offset[8]


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

            pressure_value[i] = fingertip[i] - pressure_offset[i] - j[i]
            #pressure_value[i] = pressure_previous[i]*0.85+pressure_value[i]*0.15
            pressure_previous[i]=pressure_value[i]
            temp_th[i]=fingertip[i+1]
            if pressure_value[0]<-0.1:
                pressure_value[0]=0
                zeroflag1=0
            if pressure_value[4] < -0.2:
                pressure_value[4] = 0
                zeroflag2 = 0
            if pressure_value[8] < -0.2:
                pressure_value[8] = 0
                zeroflag3 = 0

            fingertip[i] = int((pressure_value[i] / fullscale[i]) * 255)
            fingertip[i+1] = int((fingertip[i+1] / fullscale[i+1]) * 255)
            fingertip[i+2] = int((fingertip[i+2] / fullscale[i+2]) * 255)
            fingertip[i+3] = int((fingertip[i+3] / fullscale[i+3]) * 255)
            #fingertip[i] = int(fingertip[i]*0.9+fingertip[i+2]*0.1)
            if fingertip[i] < 0:
                fingertip[i] = 0
            if fingertip [i] > 255:
                fingertip[i]=255
            if fingertip [i+1] > 255:
                fingertip[i+1]=255
            if fingertip [i+2] > 255:
                fingertip[i+2]=255
            if fingertip [i+3] > 255:
                fingertip[i+3]=255


    else:
        fingertip = 0



    IMGcounter = IMGcounter + 1
    if IMGcounter == 512:
        IMGcounter = 0
    print("pressure1:", fingertip[0], "  ", "pressure2:", fingertip[4], "  ", "pressure3:", "  ", fingertip[8])

    pub0.publish(fingertip)




def listener():
    global fingertip, pub0
    while not rospy.is_shutdown():
        try:
            pub0 = rospy.Publisher('sensors/spx/scaled', numpy_msg(Floats))
            rospy.Subscriber("sensors/spx/raw", numpy_msg(Floats), callback)
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down Enhanced Grasping!")
        except IOError:
            print("Shuting down Enhanced Grasping!")


if __name__ == '__main__':
    sensorvalue = np.zeros((13, BUFF_SIZE))
    zeroflag1 = 0
    zeroflag2 = 0
    zeroflag3 = 0
    IMGcounter = 0
    fullscale=np.zeros(13)
    ind=np.array([0,4,8])
    fingertip = np.zeros(12)
    pressure_offset = np.zeros(9)
    intercept = np.zeros(10)
    pressure_zero = np.zeros(9)
    pressure_value = np.zeros(9)
    pressure_previous = np.zeros(9)
    temp_th = np.zeros(9)
    j = np.zeros(9)
    m = np.zeros(10)
    flag = np.zeros(10,dtype=bool)
    index_counter = 0
    time_start = 0
    plt.ion()
    mat_index=0
    rospy.init_node('fingertips_SPX_list', anonymous=True)
    listener()


