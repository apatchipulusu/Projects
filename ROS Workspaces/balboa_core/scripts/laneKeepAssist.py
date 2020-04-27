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

#Variables to publish robot movements
pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=0) #Tells the topic and data type of the data being published
cmd = geometry_msgs.msg.Twist()

#Path Planning Variables
distanceToTargetX = 12
distanceToTargetY = 6
pathList = []
pathIndex = 0

#Scanning Variables
finalScanList = []
tempDistance = 0
scanningRight = True
scanningNow = True
scanIndex = 0

#Reflectance Sensor Data
lineSensor0 = 0
lineSensor1 = 0
lineSensor3 = 0
lineSensor4 = 0

def pathPlanner():
    #globals
    global distanceToTargetX
    global distanceToTargetY
    global pathList
    #left/right decider init
    lastTurn = False
    #For every 3 inches to travel in the y direction, add all these commands to queue
    for x in range(distanceToTargetY/3):
	#Go straight distance x
	for x in range(0, distanceToTargetX):
	    pathList.append(1.0)
	#turn right unless previous turn was right, then turn left
	if lastTurn == False: 
	    pathList.append("Right1")#multiple maneuvers required to perform turning function
	    pathList.append("Right2")
	    pathList.append("Right3")
	    lastTurn = True
	elif lastTurn == True:
	    pathList.append("Left1")
	    pathList.append("Left2")
	    pathList.append("Left3")
	    lastTurn = False
    #Debug:
    #for x in range(0,len(pathList)):
	#print(pathList[x])

def pathSetter():
    #globals
    global pathIndex
    global pathList
    global target
    global distanceTarget
    global scanningNow
    global scanningRight
    global tempDistance
    global averageWheelDistance
    global scanIndex
    #Updates distance and angle targets based on movement to execute
    if pathIndex < len(pathList):
	if(pathList[pathIndex] == 'Right1'):
	    scanningNow = False
	    scanIndex += 4
	    target -= 83
	    #debug:
	    print("Turning Right")
	elif(pathList[pathIndex] == 'Right2'):
	    distanceTarget += 0.3498
	elif(pathList[pathIndex] == 'Right3'):
	    target -= 83
	    scanningRight = False
    	elif(pathList[pathIndex] == 'Left1'):
	    scanningNow = False
	    scanIndex += 4
	    target += 83
	    #debug:
	    print("Turning Left")
	elif(pathList[pathIndex] == 'Left2'):
	    distanceTarget += 0.3498
	elif(pathList[pathIndex] == 'Left3'):
	    target += 83
	    scanningRight = True
	else:
	    scanningNow = True
	    tempDistance = averageWheelDistance
	    distanceTarget += float(0.1166) * float(pathList[pathIndex])
	    #debug:
	    print("Going Straight")
        pathIndex += 1 #increments index so next command is executed on next run

def scannerInit():
    #globals
    global finalScanList
    global distanceToTargetY
    #initalizes the scanning of the reflectance sensors into multiple lists
    for x in range(0, distanceToTargetY/3):
	myList1 = []
	myList2 = []
	myList3 = []
	myList4 = []
	finalScanList.append(myList1)
	finalScanList.append(myList2)
	finalScanList.append(myList3)
	finalScanList.append(myList4)
    #debug:
    #print("Num Lists: " + str(len(finalScanList)))

def scanner():
    #globals
    global scanningNow
    global scanningRight
    global finalScanList
    global scanIndex
    global tempDistance
    global averageWheelDistance
    global lineSensor0
    global lineSensor1
    global lineSensor3
    global lineSensor4
    #Takes and saves sensor readings when robot reaches target distance and angle
    if scanningNow == True:
	#Write characters to scanner lists. Different case of scanning with different direction
	if scanningRight == True:
	    if lineSensor0 > 1000:
		finalScanList[scanIndex].append("M")
	    else:
		finalScanList[scanIndex].append(" ")
	    if lineSensor1 > 900:
		finalScanList[scanIndex + 1].append("M")
	    else:
		finalScanList[scanIndex + 1].append(" ")
	    if lineSensor3 > 900:
		finalScanList[scanIndex + 2].append("M")
	    else:
		finalScanList[scanIndex + 2].append(" ")
	    if lineSensor4 > 1000:
		finalScanList[scanIndex + 3].append("M")
	    else:
		finalScanList[scanIndex + 3].append(" ")
	elif scanningRight == False: 
	    if lineSensor4 > 1000:
		finalScanList[scanIndex].insert(0, "M")
	    else:
		finalScanList[scanIndex].insert(0, " ")
	    if lineSensor3 > 900:
		finalScanList[scanIndex + 1].insert(0, "M")
	    else:
		finalScanList[scanIndex + 1].insert(0, " ")
	    if lineSensor1 > 900:
		finalScanList[scanIndex + 2].insert(0, "M")
	    else:
		finalScanList[scanIndex + 2].insert(0, " ")
	    if lineSensor0 > 1000:
		finalScanList[scanIndex + 3].insert(0, "M")
	    else:
		finalScanList[scanIndex + 3].insert(0, " ")
	scanningNow = False
	#Update tempDistance to current distance
        tempDistance = averageWheelDistance
    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laneKeepAssist', anonymous=True)
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
    
    #Path planning and scanner globals
    global pathIndex
    global pathList
    global scanningNow
    pathPlanner()
    scannerInit()
    global finalScanList

    #this to continuously operate the PID
    while not rospy.is_shutdown():
	#if robot is at target, execute pathsetter to update target #Note distance diff < 0.02
	if abs(distanceTarget - averageWheelDistance) < 0.02 and abs(target - angle) < 3 and pathIndex < len(pathList): 
	    #debug: 
	    #print("Distance Diff: " + str(abs(distanceTarget - averageWheelDistance)) + " Angle Diff" + str(abs(target - angle)))
	    pathSetter()

	#Runs a task based scanner
	scanner()
	
	#print the final patten that was scanned
	if pathIndex >= len(pathList):
	    print(len(pathList))
	    for myList in finalScanList:
		for i in myList:
		    print(i),
		print()
	    exit()
	    
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
    
    #Globals for reflectance sensors
    global lineSensor0
    global lineSensor1
    global lineSensor3
    global lineSensor4
    lineSensor0 = data.lineSensor0
    lineSensor1 = data.lineSensor1
    lineSensor3 = data.lineSensor3
    lineSensor4 = data.lineSensor4

if __name__ == '__main__':
    global state
    state = False
    listener()
