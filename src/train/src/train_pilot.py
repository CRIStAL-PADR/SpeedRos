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
import sys
import signal
import rospy
from std_msgs.msg import String
from speedlib.dcc import dcc_object
from speedlib.dcc .dcc_trains import Train


def start_controller():
	"""
	Starts the controller
	"""
	dcc_object.start()

def stop_controller(signal,frame):
	"""
	Stops the controller when the user presses
	control c
	"""
	dcc_object.stop()
	sys.exit(0)

class TrainPiloteNode:
	"""
		 This class is used to create a ROS node capable of controlling
		  trains with ROS
	"""

	def __init__(self, numbof_train, start):
		"""
        Parameters
	    ----------
	    num_train : int
            It corresponds to the number of train to initialize when
            when a train node is created

        start : int
            It corresponds to the number of the first train
	    Returns
	    -------
	    None.

	    """
		self.train = {}
		for i in range(start, numbof_train):
			self.train[i] = Train("DCC"+str(i), i)

	def process_data(self, data):
		"""
        Get the command in the form of a string sent by the user and transform it
        in dictionary which will be used in by the callback method in order to execute the
        corresponding command

	    Parameters
	    ----------
	    data : String
	        Command send by the user
	    Returns
	    -------
	    data_dict : dict
	        Command return after processing
        """
		data_split = data.split(";")
		data_dict = {} 
		for i in range(len(data_split)):
			buffer = data_split[i].split(":")
			data_dict[buffer[0].strip()] = buffer[1].strip()
		return data_dict

	def callback(self, data):
		"""
	    This method is called when a message arrives on the node

	    Parameters
	    ----------
	    data : string
        """

		command = self.process_data(data.data)

		if command["train_command"] == "faster":
			self.train[int(command["train_number"])].faster()

		elif command["train_command"] == "slower":
			self.train[int(command["train_number"])].slower()

		elif command["train_command"] == "speed" and self.data.speed_value !=0:
			self.train[int(command["train_number"])].speed = int(command["speed_value"])

		elif command["train_command"] == "reverse":
			self.train[int(command["train_number"])].reverse()

		elif command["train_command"] == "f1":
			self.train[int(command["train_number"])].f1 = command["accessories_value"]

		elif command["train_command"] == "f2":
			self.train[int(command["train_number"])].f2 = command["accessories_value"]

		elif command["train_command"] == "f3":
			self.train[int(command["train_number"])].f3 = command["accessories_value"]

		elif command["train_command"] == "f4":
			self.train[int(command["train_number"])].f4 = command["accessories_value"]

		elif command["train_command"] == "fl":
			self.train[int(command["train_number"])].fl = command["accessories_value"]



if __name__=='__main__':

	start_controller() # j'excécute la fonction start_locomotive
	trainnode = TrainPiloteNode(int(sys.argv[1]), int(sys.argv[2]))
	rospy.init_node('train', anonymous=True)
	print("Initialisation du noeud")
	rospy.Subscriber("train/command", String, trainnode.callback)

	signal.signal(signal.SIGINT,stop_controller)

	rospy.spin()
