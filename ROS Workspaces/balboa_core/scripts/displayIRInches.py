#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import balboa_core.msg
from std_msgs.msg import String, UInt32

#Tells the topic and data type of the data being published
pub = rospy.Publisher('irInches', UInt32, queue_size=0) 
cmd = UInt32

#Tells the topic and data type of the data being published
pub2 = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=0) 
cmd2 = geometry_msgs.msg.Twist()

#speed constant that can be adjusted as needed
speed = 1

def callback(data):#Executes whenever new data is received
    #globals
    global pub
    global cmd
    global speed
    #Publish data to irInches topic
    if data.irInput > 69 and data.irInput < 512: #Operating range of sensor
	cmd = 10744*(data.irInput ** -0.9951)#convert from raw IR data to inches
	pub.publish(cmd)
    #Bang bang controller to respond to ir sensor input
    if data.irInput > 372: #Approx 30 inches
	cmd2.linear.x = -speed
    elif data.irInput < 372: 
	cmd2.linear.x = speed
    else: 
	cmd2.linear.x = 0
    pub2.publish(cmd2)

def listener():
    #Initialize node and subscribe to data coming in from balboaLL topic
    rospy.init_node('displayIRInches', anonymous=True)
    rospy.Subscriber("balboaLL", balboa_core.msg.balboaLL, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
