#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_forward():
	rospy.init_node('project1_pkg')
	velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
	vel_msg = Twist()

	speed = 0.5
	
	#forward speed
	vel_msg.linear.x = speed

	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0

	#This one turns in place
	vel_msg.angular.z = 0

	while not rospy.is_shutdown():
		

		velocity_publisher.publish(vel_msg)



if __name__ == '__main__':
	try:
		move_forward()
	except rospy.ROSInterruptException: pass
