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
from speedlib import dcc
from speedlib.dcc import dcc_object, dcc_trains
from speedlib.dcc.dcc_object import DCCObject
from speedlib.dcc .dcc_trains import Train
import sys
import signal

def start_controller():
	dcc_object.start()

def stop_controller(signal,frame):
	dcc_object.stop()
	sys.exit(0)

class TrainPiloteNode:
	"""
		 This class is used to create a ROS node capable of controlling
		  trains with ROS
	"""

	def __init__(self, num_train=3):
        """
	    

	    Parameters
	    ----------
	    num_train : int, optional
	        DESCRIPTION. The default is 3.
            It corresponds to the number of train to initialize when calling the constructor
            when a train node is created
	    Returns
	    -------
	    None.

	    """
		self.train = {}
		for i in range(1, num_train):
			self.train[i] = Train("DCC"+str(i), i)

	def callback(self, data):
		if not isinstance(data.train_name, str):
			raise TypeError("train_name must be a str but got" +str(data.train_name))

		if not isinstance(data.train_number, int):
			raise TypeError(" train adress must be a str but got" +str(data.train_number))
		if data.train_command == "faster":
            		self.train[data.train_number].faster()

		if data.train_command == "slower":
			self.train[self.object.train_number].slower()

		if data.train_command == "speed" and self.data.speed_value !=0:
			self.train[data.train_number].speed = data.speed_value

		if data.train_command == "reverse":
			self.train[data.train_number].reverse()

		if data.train_command == "f1":
			self.train[data.train_number].f1 = data.accessories_value

		if data.train_command == "f2":
			self.train[data.train_number].f2 = data.accessories_value

		if data.train_command == "f3":
			self.train[data.train_number].f3 = data.accessories_value

		if data.train_command == "f4":
			self.train[data.train_number].f4 = data.accessories_value

		if data.train_command == "fl":
			self.train[data.train_number].fl = data.accessories_value

		if data.train_command == "train_info":
			print(self.train[data.train_number])



if __name__=='__main__':

	start_controller() # j'excécute la fonction start_locomotive
	trainnode = TrainPiloteNode()
	rospy.init_node('train', anonymous=True)
	print("Initialisation du noeud")
	rospy.Subscriber("train/command", train_message, trainnode.callback)

	signal.signal(signal.SIGINT,stop_controller)

	rospy.spin()
