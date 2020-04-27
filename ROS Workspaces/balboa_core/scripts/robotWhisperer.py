#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from balboa_core.msg import balboaMotorSpeeds
from std_msgs.msg import String, UInt8

def callback(data):#Executes whenever new data is received
    try:
        talker(data)
    except rospy.ROSInterruptException:
        pass

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robotController', anonymous=True)

    #rospy.Subscriber("/turtle1/cmd_vel", geometry_msgs.msg.Twist, callback)
    rospy.Subscriber("cmd_vel", geometry_msgs.msg.Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def talker(data):
    #initializes the publisher
    pub = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size=0)
    cmd = balboaMotorSpeeds()
    rate = rospy.Rate(100)#High rate to increase responsiveness
    pub.publish(cmd)#publish a blank command once to increase reliability
    rate.sleep()

    #this is for linear movement
    cmd.left += int(data.linear.x * 10.0)
    cmd.right += int(data.linear.x * 10.0)

    #this is for rotational movement
    cmd.left += int(data.angular.z * -5.0)
    cmd.right += int(data.angular.z * 5.0)
    
    maxSpeed = 50

    #This sets a maximum speed that the left wheel can turn
    if cmd.left > maxSpeed:
        cmd.left = maxSpeed
    elif cmd.left < -maxSpeed:
        cmd.left = -maxSpeed

    #This sets a maximum speed that the left right can turn
    if cmd.right > maxSpeed:
        cmd.right = maxSpeed
    elif cmd.right < -maxSpeed:
        cmd.right = -maxSpeed

    #Publish 5 times to increase reliability
    for x in range(0,5):
        pub.publish(cmd)

    #Sets speed to 0 for when buttons arent pressed so that the robot stops
    cmd.left = 0.0
    cmd.right = 0.0
    pub.publish(cmd)
    rate.sleep()

if __name__ == '__main__':
    listener()
