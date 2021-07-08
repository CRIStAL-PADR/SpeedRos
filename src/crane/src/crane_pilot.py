#!/usr/bin/env python3
# coding: utf-8

#======= Import ================

import rospy
from crane.msg import crane as crane_message
from speedlib.cranes import faller
from speedlib.cranes.faller import Crane
import sys

class CranePiloteNode:

    def __init__(self, ip="172.17.217.217"):
        self.ip = ip;
        self.crane = Crane()
        self.crane.init(self.ip)
        self.motors = {"MotorCrab": 1, "MotorSpreader": 3, "MotorChassis" : 2}
        self.motors_direction = {"MotorDirectionForward": 1, "MotorDirectionBackward":-1}

    def callback(self,data):

        if data.crane_command not in ["start", "stop", "step", "start_for", "battery", "change_speed", "get_speed", "set_speed"]:
            raise RuntimeError("Erreur dans la commande adressé au moteur : "+str(data.crane_command))

        if data.motors_name not in ["MotorChassis", "MotorSpreader", "MotorCrab"]:
            raise RuntimeError("motors_name must be MotorChassis, MotorSpreader, MotorCrab but got "+str(data.motors_name))

        if not isinstance(data.values, int):
            raise TypeError("data.values must be an integer but got :"+str(data.values))

        if data.motors_direction not in ["MotorDirectionBackward", "MotorDirectionForward"]:
            raise RuntimeError("motor direction must be MotorDirectionForward or MotorDirectionBackward but got "+str(data.motors_direction))

        if data.crane_command == "start":
            self.crane.start(self.motors[data.motors_name], self.motors_direction[data.motors_direction])

        elif data.crane_command == "stop":
            self.crane.stop(self.motors[data.motors_name])

        elif data.crane_command == "step":
            self.crane.step(self.motors[data.motors_name], self.motors_direction[data.motors_direction])

        elif data.crane_command == "start_for":
            self.crane.start_for(data.values* faller.ureg.second, self.motors[data.motors_name],
                                            self.motors_direction[data.motors_direction])

        elif data.crane_command == "battery":
            print(self.crane.battery)

        elif data.crane_command == "change_speed":
            print(self.crane.change_speed(self.motors[data.motors_name], data.values))

        elif data.crane_command == "get_speed":
            print(self.crane.get_speed(self.motors[data.motors_name]))

        elif data.crane_command == "set_speed":
            print(self.crane.set_speed(self.motors[data.motors_name], data.values))



if __name__=='__main__':
    cranenode = CranePiloteNode(sys.argv[1])
    rospy.init_node(sys.argv[2], anonymous=True)
    rospy.Subscriber(sys.argv[3], crane_message, cranenode.callback)
    rospy.spin()
