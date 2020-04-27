#!/usr/bin/env python
# Created as part of HW2 for the UNL robotics class CSCE 439
# by Anirudh Patchipulusu

import rospy
from hw2stats.msg import offensivePlay, passData

#globals to store data
totalPasses = 0.0
totalPassesCompleted = 0.0
passCompletionPercent = 0.0
totalPassingYards = 0
totalRushingYards = 0

#globals to publish data
pub = rospy.Publisher('huskerOffensiveStats', passData, queue_size=1)
cmd = passData()

def callback(data):
    #Executes whenever new data is received
    global totalPasses
    global totalPassesCompleted
    global passCompletionPercent
    global totalPassingYards
    global totalRushingYards
    global pub
    global cmd
    #Store data
    if data.isPass == True:
        totalPasses += 1 
    else:
        totalRushingYards += data.yards
    if data.isPass == True and data.passCompleted == True:
        totalPassesCompleted += 1
	totalPassingYards += data.yards
    #Calculate pass completion percent
    if totalPasses != 0:
        passCompletionPercent = totalPassesCompleted/totalPasses
        #print '%.2f' % passCompletionPercent
    #populate publisher and publish passData
    cmd.passCompletionPercent = passCompletionPercent
    cmd.totalPassingYards = totalPassingYards
    cmd.totalRushingYards = totalRushingYards
    pub.publish(cmd)

def listener():
    #Initialize node and subscribe to data coming in from offensivePlay topic
    rospy.init_node('offensiveStats', anonymous=False)
    rospy.Subscriber("plays/huskerkOffensivePlay", offensivePlay, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
