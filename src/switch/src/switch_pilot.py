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
from speedlib.dcc import dcc_object, dcc_switches
from speedlib.dcc.dcc_object import DCCObject
from speedlib.dcc.dcc_switches import Switch


def start_controller():
    dcc_object.start()

def stop_controller(signal, frame):
    dcc_object.stop()
    sys.exit(0)

class SwitchPiloteNode:
    
	def __init__(self, num_switch, start):
		self.switch ={}
		for i in range(start, num_switch+1):
			self.switch[i] = Switch("DCC"+str(i), i)
		self.switch_adress_and_command = {}
		self.biais_number_and_state = {}
		
		
	def process_data(self, data):
		data_split = data.split(";")
		print("data_split : ", data_split)
		data_dict = {} 
		for i in range(len(data_split)):
			buffer = data_split[i].split(":")
			data_dict[buffer[0].strip()] = buffer[1].strip()
		return data_dict

	def callback(self, data):
		
		command = self.process_data(data.data)

		if int(command["biais_id"]) not in [1, 2]:
			raise ValueError("biais_id must be 1 or 2 but got :" +str(int(command["biais_id"])))
        
		if command["switch_command"] == "biais":
			self.switch[int(command["switch_number"])].biais = [int(command["biais_id"]), bool(command["biais_state"])]
		
		if command["switch_command"] == "biais_info":
			print(self.switch[int(command["switch_number"])].biais)

if __name__=='__main__':

   start_controller()
   switchnode = SwitchPiloteNode(int(sys.argv[1]), int(sys.argv[2]))
   rospy.init_node('switch', anonymous=True)
   print("Initialisation du noeud")
   rospy.Subscriber("switch/command", String, switchnode.callback)

   signal.signal(signal.SIGINT,stop_controller)

   rospy.spin()
