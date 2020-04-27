#!/usr/bin/env python
#
# Originally written for UNL CSCE 439/839 robotics class
# by Carrick Detweiler
#
# HW2 

import roslib
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Float64
from random import gauss

class RocketShip(object):

    def updateRocket(self):
        # Increase acceleration control input, but only to a point
        if(self.control_accel.data < 1):
            self.control_accel.data += self.control_accel_delta

        # Update the true velocity based on controlled accel plus noise
        self.true_accel = self.control_accel.data + gauss(0,0.01)
        self.true_vel.data += self.true_accel*0.1 #10Hz rate
        # Update the true altitude
        self.true_altitude.data += self.true_vel.data*0.1

        # Now update the sensed accel, which will have more noise
        self.sensed_accel = self.true_accel + gauss(0,0.01)
        # Sensed velocity with more noise
        self.sensed_vel = self.true_vel.data + gauss(0,1)


    def __init__(self):
        rospy.init_node('rocketShip')

        # Create Publishers
        self.pub_true_altitude = rospy.Publisher('groundtruth/altitude', Float64, queue_size=1)
        self.pub_true_vel = rospy.Publisher('groundtruth/velocity', Float64, queue_size=1)
        self.pub_true_accel = rospy.Publisher('groundtruth/acceleration', Float64, queue_size=1)
        self.pub_sensed_accel = rospy.Publisher('sensors/acceleration', Float64, queue_size=1)
        self.pub_sensed_vel = rospy.Publisher('sensors/velocity', Float64, queue_size=1)
        self.pub_control_accel = rospy.Publisher('control/acceleration', Float64, queue_size=1)
        
        # Init variables
        self.true_altitude = Float64()
        self.true_vel = Float64()
        self.true_accel = Float64()
        self.sensed_accel = Float64()
        self.sensed_vel = Float64()
        self.control_accel = Float64()

        # Init values
        self.control_accel_delta = 0.005
        self.true_altitude.data = 0
        self.true_vel.data = 0
        self.control_accel.data = 0
        
    def flyMyRocket(self):
        while not rospy.is_shutdown():
            self.updateRocket()

            # Now publish everything
            self.pub_true_altitude.publish(self.true_altitude)
            self.pub_true_vel.publish(self.true_vel)
            self.pub_true_accel.publish(self.true_accel)
            self.pub_sensed_accel.publish(self.sensed_accel)
            self.pub_sensed_vel.publish(self.sensed_vel)
            self.pub_control_accel.publish(self.control_accel)

            rospy.sleep(0.1)


if __name__ == '__main__':
    a = RocketShip()
    a.flyMyRocket()
