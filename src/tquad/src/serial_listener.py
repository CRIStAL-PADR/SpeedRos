#!/usr/bin/env python
# coding: utf-8

#======= Import ================
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import Range
from sensor_msgs.msg import BatteryState
from tquad.msg import LineSensor

lineSensor = LineSensor()
ultrasound = Range()
battery = BatteryState()
encoders = Int64MultiArray()

def publishRange(value):
    """
        Fonction pour publier la valeur renvoyée par le capteur ultrason
    """
    ultrasound.header.stamp = rospy.Time.now()
    ultrasound.header.frame_id = "/ultrasound"
    ultrasound.radiation_type = 0
    ultrasound.field_of_view = 0.1
    ultrasound.min_range = 0
    ultrasound.max_range = 2
    ultrasound.range = value
    ultrasound_pub.publish(ultrasound)

def publishBatteryVoltage(value):
    """
        Fonction pour publier le voltage et le pourcentage de la batterie
    """
    battery.header.stamp = rospy.Time.now()
    battery.voltage = value / 1000
    battery.percentage = value / 7400
    battery_pub.publish(battery)

def publishLineSensors(middle, left, right):
    """
        Fonction pour publier les valeurs renvoyées par les capteurs de ligne
    """
    lineSensor.left = left
    lineSensor.middle = middle
    lineSensor.right = right
    lines_pub.publish(lineSensor)

def publishEncoders(motorFrontRight, motorBackRight, motorFrontLeft, motorBackLeft) :
    """
        Fonction qui retourne la valeur des encodeurs sur le topic tquad/encoders
    """
    encoders.data = [motorFrontRight, motorBackRight, motorFrontLeft, motorBackLeft]
    encoders_pub.publish(encoders)

def callback(data):
    """
        Fonction callback du subscriber
    """
    publishRange(data.data[0])
    publishBatteryVoltage(data.data[4])
    publishLineSensors(data.data[1], data.data[2], data.data[3])
    publishEncoders(data.data[5],data.data[6],data.data[7],data.data[8])

if __name__ == '__main__':
    try:
        rospy.init_node('serial_split')
        rospy.Subscriber("tquad/serial_publisher", Float64MultiArray, callback)
        ultrasound_pub = rospy.Publisher('tquad/ultrasound', Range, queue_size=2)
        lines_pub = rospy.Publisher('tquad/lines_sensors', LineSensor, queue_size=2)
        battery_pub = rospy.Publisher('tquad/battery_state', BatteryState, queue_size=2)
        encoders_pub = rospy.Publisher('tquad/encoders', Int64MultiArray, queue_size =2)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass