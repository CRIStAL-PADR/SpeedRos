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
from switch.msg import switch as switch_message
from speedlib.dcc import dcc_object, dcc_switches
from speedlib.dcc.dcc_object import DCCObject
from speedlib.dcc.dcc_switches import Switch


def start_controller():
    dcc_object.start()

def stop_controller(signal, frame):
    dcc_object.stop()
    sys.exit(0)

class SwitchPiloteNode:
    
    def __init__(self, num_switch):
        self.switch ={}
        for i in range(1, num_switch+1):
            self.switch[i] = Switch("DCC"+str(i), i)
    
    def callback(self, data):
        print(data)

        if not isinstance(data.switch_name, str):
            raise TypeError("Switch_name must be a str but got "+str(data.switch_name))
        
        if not isinstance(data.switch_address, int):
            raise TypeError("Switch_address must be an int but got "+str(data.switch_address))

        if not isinstance(data.biais_id, int):
            raise TypeError("biais_id  be a int but got "+str(data.biais_id))
        
        if data.biais_id not in [1, 2]:
            raise ValueError("biais_id must be 1 or 2 but got :" +str(data.biais_id))
        
        if data.switch_command == "biais":
            self.switch[data.switch_address].biais = [data.biais_id ,data.biais_state]
        
        if data.switch_command == "biais_info":
            print(self.switch[data.switch_address].biais)

if __name__=='__main__':

   start_controller()
   print(type(sys.argv[1]))
   switchnode = SwitchPiloteNode(int(sys.argv[1]))
   rospy.init_node('switch', anonymous=True)
   print("Initialisation du noeud")
   rospy.Subscriber("switch/command", switch_message, switchnode.callback)

   signal.signal(signal.SIGINT,stop_controller)

   rospy.spin()
