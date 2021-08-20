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
import rospy
from std_msgs.msg import String
from speedlib.cranes import faller
from speedlib.cranes.faller import Crane

class CranePiloteNode:
	""" 
        This class is used to create a ROS node capable of controlling cranes
        with ROS
	"""

	def __init__(self, i_p="172.17.217.217"):
		"""
        Parameters
        ----------
        i_p : TYPE, optional
            DESCRIPTION. The default is "172.17.217.217".
            It corresponds to the ip address of the crane
		"""
		self.i_p = i_p
		self.crane = Crane()
		self.crane.init(self.i_p)
		self.motors = {"MotorCrab": 1, "MotorSpreader": 3, "MotorChassis" : 2}
		self.motors_direction = {"MotorDirectionForward": 1, "MotorDirectionBackward":-1}

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
		print("data_split : ", data_split)
		data_dict = {}
		for i in range(len(data_split)):
			buffer = data_split[i].split(":")
			data_dict[buffer[0].strip()] = buffer[1].strip()
		return data_dict

	def callback(self,data):
		"""
        Parameters
        ----------
        data :  crane.msg
            DESCRIPTION : It is the argument of the callback function containing 
            the messages

		"""
		command = self.process_data(data.data)
		print(command["crane_command"])
		
		if command["crane_command"] == "start":
			self.crane.start(self.motors[command["motors_name"]], 
								self.motors_direction[command["motors_direction"]])

		elif command["crane_command"] == "stop":
			self.crane.stop(self.motors[command["motors_name"]])
			
		elif command["crane_command"] == "step":
			self.crane.start(self.motors[command["motors_name"]], 
								self.motors_direction[command["motors_direction"]])
		
		elif command["crane_command"] == "start_for":
			self.crane.start_for(float(command["value"]) * faller.ureg.second,
						self.motors[command["motors_name"]], 
								self.motors_direction[command["motors_direction"]])
		elif command["crane_command"] == "battery":
			print(self.crane.battery)
			
		elif command["crane_command"] == "change_speed":
			print(self.crane.change_speed(self.motors[command["motors_name"]], int(command["speed_value"])))
		
		elif command["crane_command"] == "get_speed":
			print(self.crane.get_speed(self.motors[command["motors_name"]]))
		
		elif command["crane_command"] == "set_speed":
			print(self.crane.set_speed(self.motors[command["motors_name"]], int(command["speed_value"])))
	
if __name__=='__main__':
	cranenode = CranePiloteNode(sys.argv[1])
	print("Initialisation du noeud")
	rospy.init_node('crane', anonymous=True)
	rospy.Subscriber('crane/command', String, cranenode.callback)
	rospy.spin()
