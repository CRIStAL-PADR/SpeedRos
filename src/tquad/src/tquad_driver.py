#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class TquadDriver():
    """
        Noeud ros pour piloter les moteur de l'agv
    """
    def __init__(self):
        self.radius = 0.03
        self.lenght = 0.1
        self.width = 0.075
        #self.mpu = MPU9250(i2cbus=1, address=0x69)
        self.sub_cmd = rospy.Subscriber('tquad/cmd_vel', Twist, self.cb_cmd_vel)
        self.pub_motors = rospy.Publisher('tquad/serial_subscriber', Float32MultiArray, queue_size=1)
        #self.pub_odom = rospy.Publisher('tquad/odom', Odometry, queue_size=1, latch=True)

    def cb_cmd_vel(self, cmd):
        """
        to do something
        """
        vel_motors = self.twist2pwm( cmd.angular.z, cmd.linear.x, cmd.linear.y)
        #print(vel_motors)
        pwm_motors = Float32MultiArray(data=vel_motors)
        #print(pwm_motors)
        self.pub_motors.publish(pub_motors)

    def twist2pwm(self, wz, vx, vy):
        H = np.array([[-self.lenght - self.width, 1, -1], 
                      [ self.lenght + self.width, 1, 1], 
                      [ self.lenght + self.width, 1, -1], 
                      [ self.lenght - self.width, 1, 1]]) / self.radius
        twist = np.array([wz,vx,vy])
        twist.shape = (3,1)
        u = np.dot(H, twist)
        u = np.around(u)
        return u.flatten().tolist()
    """
    def odometry(self) :
        accel = self.mpu.readAccel()
        gyro = self.mpu.readGyro()
        time_cb = rospy.Time.now()
        odom_msg = Odometry()
        odom_msg.child_frame_id = rospy.get_namespace() + 'base_link'
        odom_msg.header.stamp = time_cb
        odom_msg.header.frame_id = rospy.get_namespace() + 'local_origin'
        odom_msg.pose.pose.orientation.x = gyro['x']
        odom_msg.pose.pose.orientation.y = gyro['y']
        odom_msg.pose.pose.orientation.z = gyro['z']

        self.pub_odom.publish(odom_msg)
    """

    def map2pw(self, value, min, max):
        return value**(max-min) + min

def main():
    rospy.init_node('tquad_driver', anonymous=True)
    tquad = TquadDriver()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass