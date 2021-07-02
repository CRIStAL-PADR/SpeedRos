#!/usr/bin/env python3
# coding: utf-8

#======= Import ================
import rospy
from train.msg import train as train_message
from speedlib.trains import dcc
from speedlib.trains.dcc import Train
import sys
import signal

def start_locomotive():
	dcc.start()

def stop_controller(signal,frame):
	dcc.stop()
	sys.exit(0)
 
class TrainPiloteNode:

	def __init__(self, num_train=3):
		self.train = {}
		for i in range(1, num_train):
			self.train[i] = Train("DCC"+str(i), i)
    		
	def callback(self, data):
		print(data)
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

	dcc.start() # j'excécute la fonction start_locomotive
	trainnode = TrainPiloteNode()						
	rospy.init_node('toto', anonymous=True)
	print("Initialisation du noeud")
	rospy.Subscriber("train/command", train_message, trainnode.callback)
	
	signal.signal(signal.SIGINT,stop_controller)

	rospy.spin()
	



