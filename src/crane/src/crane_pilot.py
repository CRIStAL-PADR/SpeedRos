#!/usr/bin/env python3
# coding: utf-8

#======= Import ================

import rospy
from crane.msg import crane as crane_message
from speedlib.cranes import faller
from speedlib.cranes.faller import Crane
from pint import UnitRegistry
import sys

class CranePiloteNode:

    def __init__(self, ip="172.17.217.217"):
        self.ip = ip;
        self.crane = Crane()
        self.crane.init(self.ip)
        self.motors = {"MotorCrab": 1, "MotorSpreader": 3, "MotorChassis" : 2}
        self.motors_direction = {"MotorDirectionForward": 1, "MotorDirectionBackward":-1}

    def callback(self,data):

        if data.motors_name in ["MotorChassis", "MotorSpreader", "MotorCrab"]:
            if data.motors_direction in ["MotorDirectionBackward", "MotorDirectionForward"]:

                if data.crane_command == "start":
                    self.crane.start(self.motors[data.motors_name], faller.data.motor_direction)

                elif data.crane_command == "stop":
                    self.crane.stop(self.motors[data.motors_name])

                elif data.crane_command == "step":
                    self.crane.step(self.motors[data.motors_name] faller.data.motors_direction)

                elif data.crane_command == "start_for":
                    self.crane.start_for(data.values*ureg.second, self.motors[data.motors_name],
                                           faller.data.motors_direction)

            elif data.crane_command == "battery":
                print(self.crane.battery)

            elif data.crane_command == "change_speed":
                self.crane.change_speed(self.motors[data.motors_name], data.values)

            elif data.crane_command == "get_speed":
                print(self.crane.get_speed(self.motors[data.motors_name]))

            elif data.crane_command == "set_speed":
                print(self.crane.set_speed(self.motors[data.motors_name], data.values))


if __name__=='__main__':
    cranenode = CranePiloteNode(sys.argv[1])
    rospy.init_node('crane', anonymous=True)
    rospy.Subscriber("crane/command", crane_message, cranenode.callback)
    rospy.spin()
