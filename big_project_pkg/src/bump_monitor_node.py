#!/usr/bin/env python

import rospy
from kobuki_msgs.msg import BumperEvent

#if bump data is received, process here
#data.bumper: LEFT (0), CENTER (1), RIGHT (2)
#data.state: RELEASED(0), PRESSED(1)

def processBump(data):
    global bump
    if (data.state == BumperEvent.PRESSED):
        bump = True
    else:
        bump = False
    rospy.loginfo("Bumper Event")
    rospy.loginfo(data.bumper)

def main():
        rospy.init_node('bump_monitor')
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, processBump)
        rospy.spin()

if __name__== '__main__':
        main()

