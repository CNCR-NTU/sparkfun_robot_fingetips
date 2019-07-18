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
__description__ = 'Subscribe SPX Fingertips sensors' 
__compatibility__ = "Python 2 and Python 3" 
__platforms__ = "Sawyer and AR10 hand" 
import rospy 
from rospy_tutorials.msg import Floats 
from std_msgs.msg import String 
import numpy as np 
import matplotlib 
import matplotlib.pyplot as plt 
import time 
 
global timing, pressure, sensorvalue, temperature, light, proximity, pressure_offset1, pressure_offset2, a 
 
 
def callback(string): 
    global timing, sensorvalue, pressure, temperature, light, proximity, pressure_offset1, pressure_offset2 
 
    buffer = string.data 
    buffer = buffer.split(',') 
    sensorvalue = np.asarray(buffer) 
    if timing == 0: 
        pressure_offset1 = float(sensorvalue[0]) 
        pressure_offset2 = float(sensorvalue[4]) 
    pressure = np.asarray([float(sensorvalue[0]) - pressure_offset1, float(sensorvalue[4]) - pressure_offset2]) 
    temperature = np.asarray([float(sensorvalue[1]), float(sensorvalue[5])]) 
    proximity = np.asarray([float(sensorvalue[2]), float(sensorvalue[6])]) 
    light = np.asarray([float(sensorvalue[3]), float(sensorvalue[7])]) 
    timing += counter 
 
 
def main(): 
    global pressure, temperature, light, proximity, a 
 
    while not rospy.is_shutdown(): 
        fig1 = plt.figure("sensor1") 
        fig1.subplots_adjust(wspace=.5) 
 
        ax1 = plt.subplot(421) 
        plt.title("SENSOR 1") 
        plt.ylabel("PRESSURE \n [hPa]") 
        plt.xlabel("time [s]") 
        plt.bar(0, pressure[0]) 
        ax1.set_ylim(0,10) 
        ax1.text(-0.1,1,a+str(int(pressure[0]))) 
 
        ax2=plt.subplot(423) 
        plt.ylabel("TEMPERATURE \n[C]") 
        ax2.set_ylim([0, 45]) 
        plt.bar(0,temperature[0]) 
        ax2.text(-0.1,1,a+str(int(temperature[0]))) 
 
        ax3=plt.subplot(425) 
        plt.ylabel("PROXIMITY") 
        ax3.set_ylim(0,10000) 
        plt.bar(0,proximity[0]) 
        ax3.text(-0.1,1,a+str(int(proximity[0]))) 
 
        ax4=plt.subplot(427) 
        plt.ylabel("LIGHT \n [lux]") 
        ax4.set_ylim(0,1000) 
        plt.bar(0, light[0]) 
        ax4.text(-0.1,1,a+str(int(light[0]))) 
 
        ax5 = plt.subplot(422) 
        plt.title("SENSOR 2") 
        plt.bar(0, pressure[1], color='cyan') 
        ax5.set_ylim(0,10) 
        ax5.text(-0.1,1,a+str(int(pressure[1]))) 
 
        ax6=plt.subplot(424) 
        ax6.set_ylim([0, 45]) 
        plt.bar(0,temperature[1],color='cyan') 
        ax6.text(-0.1,1,a+str(int(temperature[1]))) 
 
        ax7=plt.subplot(426) 
        ax7.set_ylim(0,10000) 
        plt.bar(0,proximity[1],color='cyan') 
        ax7.text(-0.1,1,a+str(int(proximity[1]))) 
 
        ax8=plt.subplot(428) 
        ax8.set_ylim(0,1000) 
        plt.bar(0, light[1],color='cyan') 
        ax8.text(-0.1,1,a+str(int(light[1]))) 

        plt.show() 
        plt.pause(0.001) 
        plt.clf() 
 
def listener(): 
    while not rospy.is_shutdown(): 
        try: 
            rospy.Subscriber("sensors/hand/spx", String, callback, queue_size=10) 
            main() 
            rospy.spin() 
        except rospy.ROSInterruptException: 
            print("Shuting down Enhanced Grasping!") 
        except IOError: 
            print("Shuting down Enhanced Grasping!") 
 
 
if __name__ == '__main__': 
    plt.ion() 
    sensorvalue = [0, 0, 0, 0, 0, 0, 0, 0] 
    pressure = [0, 0] 
    temperature = [0, 0] 
    light = [0, 0] 
    proximity = [0, 0] 
    timing = 0 
    counter = 1 
    pressure_offset1 = 0 
    pressure_offset2 = 0 
    a='Value:' 
    rospy.init_node('fingertips_SPX_list', anonymous=True) 
    listener()
