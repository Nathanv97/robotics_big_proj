#!/usr/bin/env python

import rospy
import sys
import random
import math
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan

PI = 3.1415926535897
 
class RoomScanner:

	def __init__(self):
		rospy.init_node('project1_pkg')
		rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.bumpCallback)
		rospy.Subscriber('/scan', LaserScan, self.laserscanCallback)

		self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
		self.vel_msg = Twist()

		self.bump = False
		self.laser = None

	#handles bump msgs
	def bumpCallback(self, msg):
		if(msg.state == BumperEvent.PRESSED):
			self.bump = True

	#handles the laserscan msgs
	def laserscanCallback(self, msg):
		self.laser = msg

	#This function rotates the robot 15 degrees clockwise
	def rotate(self, angleLen, CW):
		print("rotating " + str(angleLen) + " degrees")
		speed = 15
		angle = angleLen
		clockwise = CW #rotate clockwise
		
	 	
		#Converting from angles to radians
	 	angular_speed = speed * 2 * PI / 360
		relative_angle = angle * 2 * PI / 360
		
		#linear components all set to zero
		self.vel_msg.linear.x  = 0
		self.vel_msg.linear.y  = 0
		self.vel_msg.linear.z  = 0
		self.vel_msg.angular.x = 0
		self.vel_msg.angular.y = 0
		
		# Checking if our movement is CW or CCW
		if clockwise:
			self.vel_msg.angular.z = -abs(angular_speed)
		else:
			self.vel_msg.angular.z = abs(angular_speed)

		#Sleep until publisher is connected
		while(self.velocity_publisher.get_num_connections() == 0):
			rospy.sleep(0.5)		

		# Setting the current time for distance calc	
		t0 = rospy.Time.now().to_sec()
		current_angle = 0
	 
		while(current_angle < relative_angle):
									
			#print(self.vel_msg)
			self.velocity_publisher.publish(self.vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed * (t1 - t0)


		#Forcing our robot to stop
		print('done')
		self.vel_msg.angular.z = 0
		self.velocity_publisher.publish(self.vel_msg)
		rospy.sleep(1)
		#rospy.spin()
		
	def moveForward(self):
		print("moving forward 2 feet")
		speed = 0.25		
		
		#all components set to zero except linear x
		self.vel_msg.linear.x  = speed
		self.vel_msg.linear.y  = 0
		self.vel_msg.linear.z  = 0
		self.vel_msg.angular.x = 0
		self.vel_msg.angular.y = 0
		self.vel_msg.angular.z = 0

		#Sleep until publisher is connected
		while(self.velocity_publisher.get_num_connections() == 0):
			rospy.sleep(0.5)		

		# Setting the current time for distance calc	
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
	 
		while(current_distance < 0.6):
									
			#print(self.vel_msg)
			self.velocity_publisher.publish(self.vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_distance = speed * (t1 - t0)


		#Forcing our robot to stop
		print('done')
		self.vel_msg.linear.x = 0
		self.velocity_publisher.publish(self.vel_msg)

		#then make it rotate 15 degrees in random direction
		rospy.sleep(1)
		self.rotate(15, bool(random.getrandbits(1)))
		#rospy.spin()

	#This starts the scan
	def startScan(self):
		print("starting scan")
		while not rospy.is_shutdown():
			if(self.laser): #make sure laser is receiving info
				#if bumping a wall, halt
				if(self.bump):
					print("WALL HIT. STOPPING")
					self.bump = False
					self.rotate(180, True)
					#exit()
				#If wall is closer than a foot away then turn around	
				elif(min(self.laser.ranges[60:299]) < 1):
					print("wall detected in front. Turning around")
					self.rotate(180, True)
				#If a wall is detected to the right then turn 15 degrees left
				elif(min(self.laser.ranges[0:59]) < 1):
					print("wall detected to the right. turning left")
					self.rotate(15, False)
				#If a wall is detected to the left then turn 15 degrees right
				elif(min(self.laser.ranges[300:359]) < 1):
					print("wall detected to the left. turning right")
					self.rotate(15, True)
				#Else move forward
				else:
					print("starting move")
					self.moveForward()
			
 
if __name__ == '__main__':
	
	roomScanner = RoomScanner()
	rospy.sleep(1)

	roomScanner.startScan()

		


