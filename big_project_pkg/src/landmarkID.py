#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import math


def distance(x1, y1, x2, y2):
	xd = x1-x2
	yd = y1-y2
	return math.sqrt(xd*xd + yd*yd)

def callback(msg):
    	x = msg.pose.pose.position.x
    	y = msg.pose.pose.position.y
	
	# calculate the distance from the robot to all the landmarks
	CS_lab     = distance(x,y,1.2,-18.3)
	REPF       = distance(x,y,-15.9,-16.1)
	dev_atrium = distance(x,y,0.9,-4.4)
	ECE_lab    = distance(x,y,4.3,8.8)
	stairs     = distance(x,y,1.7,11.0)
	admin_off  = distance(x,y,11.7,9.7)

	# If the robot is close enough to any of the landmarks then print it out to loginfo
	if CS_lab < 1:
		rospy.loginfo('This is the CS Laboratory')
	elif REPF < 5:
		rospy.loginfo('This is the REPF')
	elif dev_atrium < 2:
		rospy.loginfo('This is Devon Hall Atrium')
	elif ECE_lab < 1:
		rospy.loginfo('This is the ECE laboratory')
	elif stairs < 1:
		rospy.loginfo('These are the stairs to the second floor')
	elif admin_off < 2:
		rospy.loginfo('This is the administration office')


def main():
	rospy.init_node('location_monitor')
	rospy.Subscriber("/odom", Odometry, callback)
	rospy.spin()

if __name__== '__main__':
	main()
