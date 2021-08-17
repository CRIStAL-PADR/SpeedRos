#!/usr/bin/env python3
# coding: utf-8
"""
    Copyright (C) 2021  CNRS
    This file is part of "Speedlib".
    "Speedlib" is an API built for the use case of autonomous navigation.
    It has  been developed to control quay cranes and trains of multimodal
    waterborne Lab as part of The SPEED project, a project which aims to
    enhance and support the growth of a system of connected port solutions,
    with the use of data science and IoT (Internet of Things) technologies.
    The library allows controlling the motion of the IoT devices at H0 scale
    in automatic mode, in three directions and exchanging with the information
    system for overall management
"""

#======= Import ================
import rospy
from train.msg import train as train_message


def train():
    pub = rospy.Publisher('train/command', train_message, queue_size=10)
    rospy.init_node('train', anonymous=True)
    rate = rospy.Rate(1) # 1hz
