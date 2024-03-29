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

#===============================================================================
# IMPORT STATEMENTS
#===============================================================================

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

#===============================================================================
# METHODS
#===============================================================================

def callback1(data):
    fingertip = np.array(data.data)
    fingertip = fingertip.reshape((12, 1, 1))
    aux1 = fingertip[0].astype(np.uint8)
    aux1 = cv2.resize(aux1, (200,200), interpolation=cv2.INTER_AREA)
    im_color1 = (cv2.applyColorMap(aux1, cv2.COLORMAP_HOT))
    cv2.imshow("fingertip1 ", im_color1)

    aux2 = fingertip[4].astype(np.uint8)
    aux2 = cv2.resize(aux2, (200,200), interpolation=cv2.INTER_AREA)
    im_color2 = (cv2.applyColorMap(aux2, cv2.COLORMAP_HOT))
    cv2.imshow("fingertip2 ", im_color2)

    aux3 = fingertip[8].astype(np.uint8)
    aux3 = cv2.resize(aux3, (200,200), interpolation=cv2.INTER_AREA)
    im_color3 = (cv2.applyColorMap(aux3, cv2.COLORMAP_HOT))
    cv2.imshow("fingertip3 ", im_color3)

    cv2.waitKey(1)

    fingertip1=np.array([fingertip[0],fingertip[1],fingertip[2],fingertip[3]])
    fingertip2=np.array([fingertip[4],fingertip[5],fingertip[6],fingertip[7]])
    fingertip3=np.array([fingertip[8],fingertip[9],fingertip[10],fingertip[11]])

    pub0.publish(fingertip1)
    pub1.publish(fingertip2)
    pub2.publish(fingertip3)

def listener():
    global pub0,pub1,pub2

    while not rospy.is_shutdown():
        try:
            pub0 = rospy.Publisher('sensors/spx/0', numpy_msg(Floats), queue_size=1)
            pub1 = rospy.Publisher('sensors/spx/1', numpy_msg(Floats), queue_size=1)
            pub2 = rospy.Publisher('sensors/spx/2', numpy_msg(Floats), queue_size=1)
            print("Sensor 0 published in topic: /sensors/spx/0.")
            print("Sensor 1 published in topic: /sensors/spx/1.")
            print("Sensor 2 published in topic: /sensors/spx/2.")
            rospy.Subscriber('sensors/spx_fingertips/raw', numpy_msg(Floats), callback1)
            

            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down the Biotac subscriber!")
#===============================================================================
#  TESTING AREA
#===============================================================================

#===============================================================================
# MAIN METHOD
#===============================================================================
if __name__ == '__main__':
    rospy.init_node('SPX_sensors_visualiser', anonymous=True)
    listener()

