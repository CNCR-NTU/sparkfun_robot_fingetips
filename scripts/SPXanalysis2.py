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
__file_name__ = 'pubFingertipSPX.py'
__description__ = 'Subscribe SPX fingertip sensors'
__compatibility__ = "Python 2 and Python 3"
__platforms__ = "Sawyer and AR10 hand"
global fingertip1

#===============================================================================
# IMPORT STATEMENTS
#===============================================================================

import rospy
import os
import cv2
import sys

from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time

global fingertip1,fingertip2,i
#===============================================================================
# METHODS
#===============================================================================

def callback1(data):
    global fingertip1
    fingertip1 = np.array(data.data)




def callback2(data):
    global fingertip2
    fingertip2 = np.array(data.data)


def main():
    global fingertip1,fingertip2,values1,values2,h,i1,i2,flag
    while not flag:
        for r in range(4):
            values1[i1,r] = fingertip1[r]
            #h[i1] = i1


            values2[i2,r] = fingertip2[r]
            #h[i2] = i2
            #print(values1[i2])
        print(values1[i1, 0])
        i2 += 1
        i1 += 1
        rospy.sleep(0.05)
        if i2 == 100:
            plt.plot(values1[:,0],'k')
            plt.plot(values1[:,2],'b')
            plt.plot(values1[:,3], 'r')

            plt.grid()
           # plt.savefig('test.png')

            plt.show()

            flag=True



def listener():
    global i1
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber('sensors/spx_fingertips/0', Floats, callback1)
            rospy.Subscriber('sensors/spx_fingertips/1', Floats, callback2)
            main()
            rospy.signal_shutdown("ok")
            rospy.spin()
        except KeyboardInterrupt:
            print("Shuting down the Biotac subscriber!")
#===============================================================================
#  TESTING AREA
#===============================================================================

#===============================================================================
# MAIN METHOD
#===============================================================================
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    i1=0
    i2=0
    values1 = np.zeros((100,4))
    values2 = np.zeros((100,4))
    h=np.zeros(100)
    fingertip1=np.zeros(4)
    fingertip2=np.zeros(4)
    flag=False


    listener()

