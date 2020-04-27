#!/usr/bin/env python
# Created as part of HW2 for the UNL robotics class CSCE 439
# by Anirudh Patchipulusu
import numpy 

import rospy
from std_msgs.msg import Float64

#globals for incoming data
sensorAcceleration = 0.0
sensorVelocity = 0.0
controlAcceleration = 0.0

#globals for truth data
actualAltitude = 0.0
actualVelocity = 0.0
actualAcceleration = 0.0

def listener():
    #Initialize node and subscribe to data coming in from rocket ship
    rospy.init_node('kalmanFilter', anonymous=False)
    rospy.Subscriber("sensors/acceleration", Float64, callback1)
    rospy.Subscriber("sensors/velocity", Float64, callback2)
    rospy.Subscriber("control/acceleration", Float64, callback3)
    rospy.Subscriber("groundtruth/altitude", Float64, callback4)
    rospy.Subscriber("groundtruth/velocity", Float64, callback5)
    rospy.Subscriber("groundtruth/acceleration", Float64, callback6)

    #rospy.spin()
    rosRate = rospy.Rate(10)#Executes at 10hz

    #globals for incoming data
    global sensorAcceleration
    global sensorVelocity
    global controlAcceleration

    #globals for truth data
    global actualAltitude
    global actualVelocity 
    global actualAcceleration

    #Matrix Initializations
    #Hardcode t = 0.1
    A = numpy.array([[1, 0.1, 0.005],[0, 1, 0.1],[0, 0, 0]])
    B = numpy.array([[0, 0, 0],[0, 0, 0],[0, 0, 1]])
    xk = numpy.array([[0],[0],[0]])
    xkMinus1 = numpy.array([[0.0],[sensorVelocity],[sensorAcceleration]])
    uk = numpy.array([[0],[0],[controlAcceleration]])
    wk = numpy.array([[0],[0],[0.01]])
    zk = numpy.array([[0],[0]])
    H = numpy.array([[0, 1, 0],[0, 0, 1]])
    vk = numpy.array([[1],[0.01]])
    xHatMinusk = numpy.array([[0],[0],[0]])
    xHatMinuskMinus1 = numpy.array([[0],[0],[0]])
    pMinusk = numpy.array([[0, 0, 0],[0, 0, 0],[0, 0, 0]])
    pkMinus1 = numpy.array([[0, 0, 0],[0, 0, 0],[0, 0, 0]])
    Q = numpy.array([[0, 0, 0],[0, 0, 0],[0, 0, 0.0001]])
    kk = numpy.array([[0, 0],[0, 0],[0, 0]])
    R = numpy.array([[1, 0],[0, 0.0001]])
    xHatk = numpy.array([[0],[0],[0]])
    pk = numpy.array([[0, 0, 0],[0, 0, 0],[0, 0, 0]])

    #Data publishing variables
    pub1 = rospy.Publisher('/kalman/altitude', Float64, queue_size=0)
    pub2 = rospy.Publisher('/kalman/velocity', Float64, queue_size=0)
    pub3 = rospy.Publisher('/kalman/acceleration', Float64, queue_size=0)
    pub4 = rospy.Publisher('/diffAltitude', Float64, queue_size=0)
    pub5 = rospy.Publisher('/diffVelocity', Float64, queue_size=0)
    pub6 = rospy.Publisher('/diffAcceleration', Float64, queue_size=0)

    while not rospy.is_shutdown():
	#Update with new data
	xkMinus1.itemset(1, sensorVelocity)
	xkMinus1.itemset(2, sensorAcceleration)
	uk.itemset(2, controlAcceleration)
	#Process
	xk = numpy.add(numpy.dot(A, xkMinus1), numpy.add(numpy.dot(B, uk), wk))
	#Measurement
	zk = numpy.add(numpy.dot(H, xk), vk)
	#Time Update
	xHatMinusk = numpy.add(numpy.dot(A, xHatMinuskMinus1), numpy.dot(B, uk))
	pMinusk = numpy.add(numpy.dot(A, numpy.dot(pkMinus1, numpy.transpose(A))), Q)
	#Measurement Update
	kk = numpy.dot(numpy.dot(pMinusk, numpy.transpose(H)), numpy.linalg.inv(numpy.add(numpy.dot(H, numpy.dot(pMinusk, numpy.transpose(H))), R)))
	xHatk = numpy.add(xHatMinusk, numpy.dot(kk, numpy.subtract(zk, numpy.dot(H, xHatMinusk))))
	pk = numpy.dot(numpy.subtract(numpy.identity(3), numpy.dot(kk, H)), pMinusk)
	#Current to past
	xkMinus1 = xk
	xHatMinuskMinus1 = xHatMinusk
	pkMinus1 = pMinusk

	#debug 
	#print xHatk

	#Publish data
	pub1.publish(xHatk[0])
	pub2.publish(xHatk[1])
	pub3.publish(xHatk[2])
	pub4.publish(abs(xHatk[0] - actualAltitude))
	pub5.publish(abs(xHatk[1] - actualVelocity))
	pub6.publish(abs(xHatk[2] - actualAcceleration))
	
	#Wait specified time before continuing loop
	rosRate.sleep()

def callback1(data):
    #Executes whenever new data is received
    global sensorAcceleration
    sensorAcceleration = data.data

def callback2(data):
    #Executes whenever new data is received
    global sensorVelocity
    sensorVelocity = data.data

def callback3(data):
    #Executes whenever new data is received
    global controlAcceleration
    controlAcceleration = data.data

def callback4(data):
    #Executes whenever new data is received
    global actualAltitude
    actualAltitude = data.data

def callback5(data):
    #Executes whenever new data is received
    global actualVelocity
    actualVelocity = data.data

def callback6(data):
    #Executes whenever new data is received
    global actualAcceleration
    actualAcceleration = data.data

if __name__ == '__main__':
    listener()
