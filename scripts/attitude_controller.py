#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
		PUBLICATIONS            SUBSCRIPTIONS
		/roll_error             /pid_tuning_altitude
		/pitch_error            /pid_tuning_pitch
		/yaw_error              /pid_tuning_roll
		/edrone/pwm             /edrone/imu/data
								/edrone/drone_command

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tfthrottle
import numpy as np
from math import sin, cos, pi, sqrt

class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		rospy.init_node('attitude_controller')  # initializing ros node with name drone_control
		self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
		self.drone_orientation_euler = [0.0, 0.0, 0.0]
		self.setpoint_cmd = [1500, 1500, 1750.0]
		self.setpoint_euler = [0.0, 0.0, 0.0]
		self.pwm_cmd = prop_speed()
		self.pwm_cmd.prop1 = 0.0
		self.pwm_cmd.prop2 = 0.0
		self.pwm_cmd.prop3 = 0.0
		self.pwm_cmd.prop4 = 0.0

		self.Kp = [55, 55, 500]
		self.Ki = [0.04, 0.04, 0.22]
		self.Kd = [36, 36, 58]
		self.setpoint_throttle = 0
		self.throttle = 0
		self.proportional = [0.0, 0.0, 0.0]
		self.integral = [0.0, 0.0, 0.0]
		self.derivative = [0.0, 0.0, 0.0]
		self.pry_cmd = [0.0, 0.0, 0.0]
		self.prev_error = [0, 0, 0]
		self.pwm_clamp_min =  [0, 0, 0, 0]
		self.pwm_clamp_max = [1024, 1024, 1024, 1024]
		self.error = [0.0, 0.0, 0.0]
		self.error_msg = [Float32(), Float32(), Float32()]

		self.sample_time = 0.0166  # in seconds

		# Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
		self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
		self.pitch_err_pub = rospy.Publisher("/pitch_error", Float32, queue_size=1)
		self.roll_err_pub = rospy.Publisher("/roll_error", Float32, queue_size=1)
		self.yaw_err_pub = rospy.Publisher("/yaw_error", Float32, queue_size=1)

		rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
		rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
		# rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
		# rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
		# rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)

	def imu_callback(self, msg):
		self.drone_orientation_quaternion[0] = msg.orientation.x
		self.drone_orientation_quaternion[1] = msg.orientation.y
		self.drone_orientation_quaternion[2] = msg.orientation.z
		self.drone_orientation_quaternion[3] = msg.orientation.w

	def drone_command_callback(self, msg):
		self.setpoint_cmd[0] = msg.rcPitch

		# ---------------------------------------------------------------------------------------------------------------
		self.setpoint_cmd[1] = msg.rcRoll
		self.setpoint_cmd[2] = msg.rcYaw
		self.throttle = ((msg.rcThrottle-1000)/1000.0*1024) + 0.0

	# Callback function for /pid_tuning_roll
	# This function gets executed each time when /tune_pid publishes /pid_tuning_roll
	# def roll_set_pid(self, roll):
	# 	self.Kp[1] = roll.Kp  # This is just for an example. You can change the ratio/fraction value accordingly
	# 	self.Ki[1] = roll.Ki*0.01
	# 	self.Kd[1] = roll.Kd*0.3

	# # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
	# def pitch_set_pid(self, pitch):
	# 	self.Kp[0] = pitch.Kp  # This is just for an example. You can change the ratio/fraction value accordingly
	# 	self.Ki[0] = pitch.Ki*0.01
	# 	self.Kd[0] = pitch.Kd*0.3
	
	# def yaw_set_pid(self, yaw):
	# 	self.Kp[2] = yaw.Kp  # This is just for an example. You can change the ratio/fraction value accordingly
	# 	self.Ki[2] = yaw.Ki*0.01
	# 	self.Kd[2] = yaw.Kd*0.3

	# ----------------------------------------------------------------------------------------------------------------------
	def imuYaw2bearing(self, imuYaw):
		return (2*pi - imuYaw - pi/2) % (2*pi)

	def pid(self):
		# Converting quaternion to euler angles
		(self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
		self.drone_orientation_euler[2] = self.imuYaw2bearing(self.drone_orientation_euler[2])

		# Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for pitch axis
		self.setpoint_euler[0] = ((self.setpoint_cmd[0]-1000)/1000.0*(2*0.1745)) - 0.1745

		# Complete the equations for roll and yaw axis
		self.setpoint_euler[1] = ((self.setpoint_cmd[1]-1000)/1000.0*(2*0.1745)) - 0.1745
		self.setpoint_euler[2] = ((self.setpoint_cmd[2]-1000)/1000.0*(2*pi)) + 0.0

		#Error Calulation
		self.error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0]
		self.error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1]

		if sin(self.setpoint_euler[2] - self.drone_orientation_euler[2]) > 0:
			self.error[2] = min(abs(self.setpoint_euler[2] - self.drone_orientation_euler[2]), abs((self.setpoint_euler[2] + 2*pi) - self.drone_orientation_euler[2]))
		else:
			self.error[2] = -min(abs(self.setpoint_euler[2] - self.drone_orientation_euler[2]), abs(self.setpoint_euler[2] - (self.drone_orientation_euler[2] + 2*pi)))

		self.error_msg[0].data = self.error[0]
		self.error_msg[1].data = self.error[1]
		self.error_msg[2].data = self.error[2]

		self.pitch_err_pub.publish(self.error_msg[0])
		self.roll_err_pub.publish(self.error_msg[1])
		self.yaw_err_pub.publish(self.error_msg[2])
			
		#PID In
		self.proportional = np.array(self.error)
		self.integral += np.array(self.error) * np.array(self.sample_time)
		self.derivative = (np.array(self.error) - np.array(self.prev_error))/self.sample_time
		self.prev_error = self.error[:]

		#PID Out
		self.pry_cmd = np.array(self.Kp)*self.proportional + np.array(self.Ki)*self.integral + np.array(self.Kd)*self.derivative
		
		#PWM set
		#						throttle		  pitch				roll			  yaw
		self.pwm_cmd.prop1 = self.throttle + self.pry_cmd[0] - self.pry_cmd[1] + self.pry_cmd[2]
		self.pwm_cmd.prop2 = self.throttle - self.pry_cmd[0] - self.pry_cmd[1] - self.pry_cmd[2]
		self.pwm_cmd.prop3 = self.throttle - self.pry_cmd[0] + self.pry_cmd[1] + self.pry_cmd[2]
		self.pwm_cmd.prop4 = self.throttle + self.pry_cmd[0] + self.pry_cmd[1] - self.pry_cmd[2]
		
		#Clamp pwm_cmd
		if(self.pwm_cmd.prop1 < self.pwm_clamp_min[0]):
			self.pwm_cmd.prop1 = self.pwm_clamp_min[0]
		elif(self.pwm_cmd.prop1 > self.pwm_clamp_max[0]):
			self.pwm_cmd.prop1 = self.pwm_clamp_max[0]
		
		if(self.pwm_cmd.prop2 < self.pwm_clamp_min[1]):
			self.pwm_cmd.prop2 = self.pwm_clamp_min[1]
		elif(self.pwm_cmd.prop2 > self.pwm_clamp_max[1]):
			self.pwm_cmd.prop2 = self.pwm_clamp_max[1]
		
		if(self.pwm_cmd.prop3 < self.pwm_clamp_min[2]):
			self.pwm_cmd.prop3 = self.pwm_clamp_min[2]
		elif(self.pwm_cmd.prop3 > self.pwm_clamp_max[2]):
			self.pwm_cmd.prop3 = self.pwm_clamp_max[2]
		
		if(self.pwm_cmd.prop4 < self.pwm_clamp_min[3]):
			self.pwm_cmd.prop4 = self.pwm_clamp_min[3]
		elif(self.pwm_cmd.prop4 > self.pwm_clamp_max[3]):
			self.pwm_cmd.prop4 = self.pwm_clamp_max[3]
		
		rospy.loginfo(self.pwm_cmd)
		self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(1/e_drone.sample_time)
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
