#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import balboa_core.msg
from std_msgs.msg import String, UInt8

# globals for target values and calculating previous error
target = 0.0
angle = 0.0
prevError = 0.0

#globals for distance PID
distanceTarget = 0.0 # target value
averageWheelDistance = 0.0 # average distance of the two wheels
previousDistanceError = 0.0 # previous distance value error
#P:1.4 D:0.25

pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=0) #Tells the topic and data type of the data being published
cmd = geometry_msgs.msg.Twist()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('IRDistanceFollower', anonymous=True)
    #Subscribe to the balboaLL topic to receive and read robot data
    rospy.Subscriber("balboaLL", balboa_core.msg.balboaLL, setAngleGlobal)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    r = rospy.Rate(10) # 20hz

    #globals for angle PID
    global pub
    global cmd

    #globals for distance PID
    global P
    global D
    global prevError
    global angle

    #globals for distance PID
    global PDistance
    global DDistance
    global previousDistanceError
    global averageWheelDistance

    #Update PID values from launch file
    PInit = rospy.search_param('P')
    P = rospy.get_param(PInit)
    DInit = rospy.search_param('D')
    D = rospy.get_param(DInit)
    PDistanceInit = rospy.search_param('PDistance')
    PDistance = rospy.get_param(PDistanceInit)
    DDistanceInit = rospy.search_param('DDistance')
    DDistance = rospy.get_param(DDistanceInit)


    #this to continuously operate the PID
    while not rospy.is_shutdown():
        #this sets the left/right rotation command
        cmd.angular.z = P * (target - angle) + D * (target - angle - prevError)
        prevError = target - angle

        #sets the forward/backward command
        cmd.linear.x = PDistance * (distanceTarget - averageWheelDistance) + DDistance * (distanceTarget - averageWheelDistance - previousDistanceError)
        previousDistanceError = distanceTarget - averageWheelDistance

        #this publishes the command
        pub.publish(cmd)
        r.sleep()

def setAngleGlobal(data):
    # the state is a boolean to initialize the targets
    global state

    #globals for angle
    global angle
    global target

    #globals for distance
    global averageWheelDistance
    global distanceTarget

    #this is to deal with the forwards and backwards
    averageWheelDistance = (data.distanceLeft + data.distanceRight) / 2000.0 #average distance of the two wheels

    #this is to set the angle value
    angle = data.angleX/1000

    #this is to set the target equal to angle for the first run.  It will only run once
    if state == False:
        target = angle
        distanceTarget = averageWheelDistance
        state = True

if __name__ == '__main__':
    global state
    state = False
    listener()
