#!/usr/bin/env python3
# coding: utf-8

#======= Import ================
import rospy
from train.msg import train
from speedlib.trains import dcc
from speedlib.trains.dcc import Train


    
class TrainPiloteNode:

    def __init__(self):
        self.object = None
    

    def train_init(self):
        train = {}
        MAX_NUMBER = 20
        for i in range(1, MAX_NUMBER):
            train[i] = Train("DCC"+str(i), i)
        return train

    def callback(self, data):

        print(self.data)
        if not isinstance(self.data.train_name, str):
            raise TypeError("train_name must be a str but got" +str(self.data.train_name))

        if not isinstance(self.data.train_number, int):
            raise TypeError(" train adress must be a str but got" +str(self.data.train_number))

        self.object = data  
    
    def start_locomotive(self):
        dcc.start

    def stop_locomotive(self):
        dcc.stop()

    def listener(self):
        train= {}
        train= self.train_init() 

        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("train/command", train, self.callback)
        
        if self.object.train_command == "faster":
            train[self.object.train_number].faster()

        if self.data.train_command == "slower":
            train[self.object.train_number].slower()

        if self.data.train_command == "speed" and self.data.speed_value !=0:
            train[self.object.train_number].speed = self.object.speed_value
        
        if self.data.train_command == "reverse":
            train[self.object.train_number].reverse()
        
        if self.data.train_command == "f1":
            train[self.object.train_number].f1 = self.object.accessories_value
        
        if self.data.train_command == "f2":
            train[self.object.train_number].f2 = self.object.accessories_value

        if self.data.train_command == "f3":
            train[self.object.train_number].f3 = self.object.accessories_value
    
        if self.data.train_command == "f4":
            train[self.object.train_number].f4 = self.object.accessories_value

        if self.data.train_command == "fl":
            train[self.object.train_number].fl = self.object.accessories_value 

        if self.data.train_command == "train_info":
            print(train[self.object.train_number])       

        rospy.spin()

if __name__=='__main__':

    TrainPiloteNode.start_locomotive()
    TrainPiloteNode.listener()
    TrainPiloteNode.stop_locomotive