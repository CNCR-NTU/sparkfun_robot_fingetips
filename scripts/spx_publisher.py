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
__file_name__ = 'getSPX.py'
__description__ = 'Publishes the SPX fingertip sensor values in the topics'
__compatibility__ = "Python 2 and Python 3"
__platforms__ = "Sawyer and AR10 hand"

#===============================================================================
# IMPORT STATEMENTS
#===============================================================================
import rospy
import serial
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

def main(pub):
    ser = serial.Serial(port='/dev/ttyUSB0',baudrate=38400,timeout=0.25)
    ser.flush();
    print("connected to: " + ser.portstr + ", baudrate: "+str(ser.baudrate))
    while not rospy.is_shutdown():
        while ser.in_waiting:
            buffer=ser.readline()
            buffer = buffer.split(',')
            if len(buffer)>12:
                mat = np.asarray(buffer[0:len(buffer)-1], dtype=np.float32)
                pub.publish(mat.flatten('F'))
    ser.close()


def publisher():
    global flag, out
    while not rospy.is_shutdown():
        try:
            pub = rospy.Publisher('sensors/spx/raw', numpy_msg(Floats), queue_size=1)
            print("SPX sensor 0 published in topic: sensors/spx/raw.")
            main(pub)
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down the spx subscriber!")

#===============================================================================
#  TESTING AREA
#===============================================================================

#===============================================================================
# MAIN METHOD
#===============================================================================
if __name__ == '__main__':
    print("[Initialising Fingertips...]\n")
    rospy.init_node('fingertips_spx', anonymous=True)
    publisher()


