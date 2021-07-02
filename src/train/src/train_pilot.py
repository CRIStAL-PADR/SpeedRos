#!/usr/bin/env python3
# coding: utf-8

#======= Import ================
import rospy
from train.msg import train as train_message
from speedlib.trains import dcc
from speedlib.trains.dcc import Train


def start_locomotive():
	dcc.start()

def stop_locomotive():
	dcc.stop()

    
class TrainPiloteNode:

	def __init__(self):
		self.object = None
		self.data = None
		self.train = {}
    

	def train_init(self):
		MAX_NUMBER = 3
		for i in range(1, MAX_NUMBER):
			self.train[i] = Train("DCC"+str(i), i)
		

	def callback(self, data):

		print(data)
		self.object = data
		self.data = data
		if not isinstance(data.train_name, str):
			raise TypeError("train_name must be a str but got" +str(data.train_name))

		if not isinstance(data.train_number, int):
			raise TypeError(" train adress must be a str but got" +str(data.train_number))
		if self.object.train_command == "faster":
            		self.train[self.object.train_number].faster()

		if self.data.train_command == "slower":
			self.train[self.object.train_number].slower()

		if self.data.train_command == "speed" and self.data.speed_value !=0:
			self.train[self.object.train_number].speed = self.object.speed_value
        
		if self.data.train_command == "reverse":
			self.train[self.object.train_number].reverse()
        
		if self.data.train_command == "f1":
			self.train[self.object.train_number].f1 = self.object.accessories_value
        
		if self.data.train_command == "f2":
			self.train[self.object.train_number].f2 = self.object.accessories_value

		if self.data.train_command == "f3":
			self.train[self.object.train_number].f3 = self.object.accessories_value
    
		if self.data.train_command == "f4":
			self.train[self.object.train_number].f4 = self.object.accessories_value

		if self.data.train_command == "fl":
			self.train[self.object.train_number].fl = self.object.accessories_value 

		if self.data.train_command == "train_info":
			print(self.train[self.object.train_number])


	def listener(self):              
	                                                                #je crée une variablee train qui va pointer vers une zone mémoire ou j'initialise un 													doctionnaire vide
		self.train_init() 							  #J'initialise tous les trains et je fais pointer la variable train crée précédement 													vers la  zone mémoire pointer par la fonction

		rospy.init_node('listener', anonymous=True)				

		rospy.Subscriber("train/command", train_message, self.callback) 

		rospy.spin()

if __name__=='__main__':

	start_locomotive() # j'excécute la fonction start_locomotive
	train = TrainPiloteNode() # j'instancie un objet de type trainPiloteNode et je fais pointer la variable train vers la zone mémoir eou se trouve l'objet instancié
	train.listener() # J
	stop_locomotive()
