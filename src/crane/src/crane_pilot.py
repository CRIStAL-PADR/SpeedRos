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
from crane.msg import crane as crane_message
from speedlib.cranes import faller
from speedlib.cranes.faller import Crane

class CranePiloteNode:
    """ Cette classe permet de créer un noeud ROS capable de piloter les grues
         avec ROS
	"""

    def __init__(self, i_p="172.17.217.217"):
        """
        Parameters
        ----------
        i_p : TYPE, optional
            DESCRIPTION. The default is "172.17.217.217".

        Returns
        -------
        None.

        """
        self.i_p = i_p
        self.crane = Crane()
        self.crane.init(self.i_p)
        self.motors = {"MotorCrab": 1, "MotorSpreader": 3, "MotorChassis" : 2}
        self.motors_direction = {"MotorDirectionForward": 1, "MotorDirectionBackward":-1}

    def callback(self,data):
        """
        Parameters
        ----------
        data :  crane.msg
            DESCRIPTION : C'est largument de la fonction callback contenant les messages

        Returns
        -------
        None.

        """

        if data.crane_command not in ["start", "stop", "step", "start_for",
                                      "battery", "change_speed", "get_speed", "set_speed"]:
            raise RuntimeError("Erreur dans la commande adressé au moteur : "+
                               str(data.crane_command))

        if data.motors_name not in ["MotorChassis", "MotorSpreader", "MotorCrab"]:
            raise RuntimeError("""motors_name must be MotorChassis, MotorSpreader,
                               MotorCrab but got """+str(data.motors_name))

        if not isinstance(data.values, int):
            raise TypeError("data.values must be an integer but got :"+str(data.values))

        if data.motors_direction not in ["MotorDirectionBackward", "MotorDirectionForward"]:
            raise RuntimeError("""motor direction must be MotorDirectionForward or
                               MotorDirectionBackward but got """+str(data.motors_direction))

        if data.crane_command == "start":
            self.crane.start(self.motors[data.motors_name],
                             self.motors_direction[data.motors_direction])

        elif data.crane_command == "stop":
            self.crane.stop(self.motors[data.motors_name])

        elif data.crane_command == "step":
            self.crane.step(self.motors[data.motors_name],
                            self.motors_direction[data.motors_direction])

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
    if len(sys.argv) <4 :
        raise RuntimeError(""" To run crane_pilote needs 3 input arguments:
                               ip address, the message and the command""")

    cranenode = CranePiloteNode(sys.argv[1])
    rospy.init_node(sys.argv[2], anonymous=True)
    rospy.Subscriber(sys.argv[3], crane_message, cranenode.callback)
    rospy.spin()
