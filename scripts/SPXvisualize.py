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
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
import datetime as dt

def callback(string):
    buffer = string.data
    buffer=strip_non_ascii(buffer)
    buffer=buffer.split(',')
    string=np.asarray(buffer)

    press1=float(string[0])
    press2=float(string[4])
    temp1=float(string[1])
    temp2=float(string[5])
    prox1=float(string[2])
    prox2=float(string[6])
    lig1=float(string[3])
    lig2=float(string[7])
    rospy.loginfo("Press1: %.2f    Press2: %.2f   Temp1: %.2f   Temp2: %.2f    Prox1: %.2f   Prox2: %.2f   Lig1: %.2f   Lig2: %.2f",press1,press2,temp1,temp2,prox1,prox2,lig1,lig2)
   # plt.plot()
   # plt.pause(0.0001)
   # plt.show()



def listener():

    # plt.ion()
    rospy.init_node('fingertips_SPX_list', anonymous=True)
    rospy.Subscriber("sensors/hand/spx", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':

    listener()
