#!/usr/bin/env python
#
# Originally written for UNL CSCE 439/839 robotics class
# by Carrick Detweiler
#
# HW2 

import roslib
roslib.load_manifest('hw2stats')
import rospy
from hw2stats.msg import offensivePlay
from std_msgs.msg import Header
from random import randint

def publishStats():
    pub = rospy.Publisher('plays/huskerkOffensivePlay', offensivePlay, queue_size=1)
    rospy.init_node('rawStats')
    while not rospy.is_shutdown():
        play = offensivePlay()
        play.header.stamp = rospy.Time.now()
        if(randint(0,1)):
            play.isPass = True
        else:
            play.isPass = False
        if(randint(0,1)):
            play.passCompleted = True
        else:
            play.passCompleted = False
        play.yards = randint(1,90)
        if(randint(0,15)):
            play.touchdown = False
        else:
            play.touchdown = True
        pub.publish(play)
        rospy.sleep(0.2)


if __name__ == '__main__':
    try:
        publishStats()
    except rospy.ROSInterruptException:
        pass
