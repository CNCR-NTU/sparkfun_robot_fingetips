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
__file_name__ = 'visualise_SPX_sensor.py'
__description__ = 'Visualise SPX Fingertips sensors'
__compatibility__ = "Python 2 and Python 3"
__platforms__ = "Sawyer and AR10 hand"
import rospy
import cv2
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import numpy as np

visualisationFlag=True
scale_percent=8000


def callback_spx(data,publishers):
    global matrix, mat_index
    mat=data.data
    #print(mat)
    for sensor in range(0, 3):
        aux=np.zeros((4,4),dtype=np.uint8)
        publishers[sensor].publish(np.asarray(mat[sensor*4:sensor*4+4], dtype=np.float32).flatten('F'))
        data_aux=mat[sensor*4:sensor*4+4]
        print(data_aux)
        aux[0, 0] = data_aux[0].astype(np.uint8)
        aux[1, 1] = data_aux[1].astype(np.uint8)
        aux[2, 2] = data_aux[2].astype(np.uint8)
        aux[3, 3] = data_aux[3].astype(np.uint8)
        if visualisationFlag:
            width = int(aux.shape[1] * scale_percent / 100)
            height = int(aux.shape[0] * scale_percent / 100)
            dim = (width, height)
            # resize image
            aux = cv2.resize(aux, dim, interpolation=cv2.INTER_AREA)
            im_color = (cv2.applyColorMap(aux, cv2.COLORMAP_HOT))
            cv2.imshow("Sensor " + str(sensor+1) , im_color)

        if visualisationFlag and cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('Quit')
            cv2.destroyAllWindows()


def listener():
    while not rospy.is_shutdown():
        try:
            pub0 = rospy.Publisher('sensors/spx/0', numpy_msg(Floats), queue_size=1)
            pub1 = rospy.Publisher('sensors/spx/1', numpy_msg(Floats), queue_size=1)
            pub2 = rospy.Publisher('sensors/spx/2', numpy_msg(Floats), queue_size=1)
            rospy.Subscriber("sensors/spx/scaled", numpy_msg(Floats), callback_spx, ([pub0, pub1, pub2]))
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down Enhanced Grasping!")
        except IOError:
            print("Shuting down Enhanced Grasping!")


if __name__ == '__main__':
    rospy.init_node('fingertips_SPX_list', anonymous=True)
    listener()


