#!/usr/bin/env python
from vitarana_drone.msg import *
import rospy
import pygame
import time

def publisher():
	rospy.init_node("publisher", anonymous=True)
	pub = rospy.Publisher("edrone/pwm", prop_speed, queue_size=1)
	msg = prop_speed()
	rate = rospy.Rate(10)

	speed = 400
	roll = 0
	pitch = 0
	yaw = 0

	pygame.init()
	done = False
	clock = pygame.time.Clock()
	pygame.joystick.init()
	joystick_count = pygame.joystick.get_count()
	
	joystick_mode = True
	
	while not done:
		for event in pygame.event.get():
			if event.type ==pygame.QUIT:
				done = True
			
			if event.type == pygame.JOYBUTTONDOWN and event.button == 9:
				if joystick_mode:
					joystick_mode = False
				elif not joystick_mode:
					joystick_mode = True
			
			# if event.type == pygame.JOYBUTTONDOWN and event.button == 7:
			# 	yaw
			# elif event.type == pygame.JOYBUTTONDOWN and event.button == 6:
			# 	speed = speed - 10

		if joystick_mode:
			joystick = pygame.joystick.Joystick(0)
			joystick.init()
			
			axis0 = joystick.get_axis(0)
			axis1 = joystick.get_axis(1)
			axis2 = joystick.get_axis(2)
			axis3 = joystick.get_axis(3)
			print(axis0)
			speed = axis3*-1023
			pitch = axis1*-50
			roll = axis0*50
			yaw = axis2*1023

		if speed<0:
			speed = 0
		elif speed>1023:
			speed = 1023
		msg.prop1 = speed - roll - pitch + yaw
		msg.prop2 = speed - roll + pitch - yaw
		msg.prop3 = speed + roll + pitch + yaw
		msg.prop4 = speed + roll - pitch - yaw

		# if axis1 < 0:
		# 	msg.prop2 = msg.prop2 - pitch
		# 	msg.prop3 = msg.prop3 - pitch
		# elif axis1 > 0:
		# 	msg.prop1 = msg.prop1 + pitch
		# 	msg.prop4 = msg.prop4 + pitch

		# if axis0 < 0:
		# 	msg.prop1 = msg.prop1 - roll
		# 	msg.prop2 = msg.prop2 - roll
		# elif axis0 > 0:
		# 	msg.prop3 = msg.prop3 + roll
		# 	msg.prop4 = msg.prop4 + roll
		
		# if axis2 < 0.:
		# 	msg.prop2 = msg.prop2 - yaw
		# 	msg.prop4 = msg.prop4 - yaw
		# elif axis2 > 0:
		# 	msg.prop1 = msg.prop1 + yaw
		# 	msg.prop3 = msg.prop3 + yaw
		

		rospy.loginfo(msg)
		pub.publish(msg)

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass