#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import balboa_core.msg
from std_msgs.msg import String, UInt8

def callback(data):#Executes whenever new data is received
     #sensorLimit0 = 1000
     #sensorLimit1 = 900
     #sensorLimit2 = 800
     #sensorLimit3 = 900
     #sensorLimit4 = 1000

     #Pull reflectance sensor limits from launch file
     sensorLimit0Init = rospy.search_param('sensorLimit0')
     sensorLimit0 = rospy.get_param(sensorLimit0Init)
     sensorLimit1Init = rospy.search_param('sensorLimit1')
     sensorLimit1 = rospy.get_param(sensorLimit1Init)
     sensorLimit2Init = rospy.search_param('sensorLimit2')
     sensorLimit2 = rospy.get_param(sensorLimit2Init)
     sensorLimit3Init = rospy.search_param('sensorLimit3')
     sensorLimit3 = rospy.get_param(sensorLimit3Init)
     sensorLimit4Init = rospy.search_param('sensorLimit4')
     sensorLimit4 = rospy.get_param(sensorLimit4Init)

     #Check if sensor data exeeds limit and print to screen accordingly
     if data.lineSensor0 > sensorLimit0: 
	rospy.loginfo("Sensor 1 over line")
     if data.lineSensor1 > sensorLimit1:
	rospy.loginfo("Sensor 2 over line")
     if data.lineSensor2 > sensorLimit2:
	rospy.loginfo("Sensor 3 over line")
     if data.lineSensor3 > sensorLimit3:
	rospy.loginfo("Sensor 4 over line")
     if data.lineSensor4 > sensorLimit4:
	rospy.loginfo("Sensor 5 over line")

def listener():
     #Initialize the node
     rospy.init_node('lineDetectorTest', anonymous=True)
     #Subscribe to robot data
     rospy.Subscriber("balboaLL", balboa_core.msg.balboaLL, callback)
     #Wait until new data comes in to execute callback
     rospy.spin()

if __name__ == '__main__':
    listener()	
