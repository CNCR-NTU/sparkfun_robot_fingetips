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
from std_msgs.msg import String
import numpy as np
import cv2
import os
import serial
import struct

def main(pub1):
    global flag
    ser = serial.Serial(port='/dev/ttyUSB0',baudrate=9600,timeout=0.25)
    print("connected to: " + ser.portstr + ", baudrate: "+str(ser.baudrate))
    while flag:
        while ser.in_waiting:  
            string=ser.readline()
            pub1.publish(string) #Format: "pressure1,temp1,proximity1,light1,pressure2,temp2,proximity2,light2,pressure3,temp3,proximity3,light3"

    ser.close()


def publisher():
    global flag, out
    while not rospy.is_shutdown():
        try:
            pub = rospy.Publisher('sensors/hand/spx', String, queue_size=1)
            print("spx published in topic: sensors/hand/spx.")
            flag=True
            main(pub)
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down the spx subscriber!")
            flag=False

#===============================================================================
#  TESTING AREA
#===============================================================================

#===============================================================================
# MAIN METHOD
#===============================================================================
if __name__ == '__main__':
    print("[Initialising Fingertips...]\n")
    rospy.init_node('fingertips_SPX_pub', anonymous=True)
    publisher()

