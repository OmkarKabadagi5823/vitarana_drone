#!/usr/bin/env python
'''
Publishers:
	/drone_command 
		msg type: vitarana_drone/edrone_cmd
	
Subcribers:
	/edrone/gps
		msg type: sensor_msgs/NavSatFix
	/edrone/gps_velocity
		msg type: geometry_msgs/Vector3Stamped
	/edrone/imu/data
		msg type: sensor_msgs/Imu
	/pid_tuning
		msg type: pid_tune/PidTune
	/edrone_controller
		msg type: vitarana_drone/edrone_pos_cmd (my custom message for setting position setpoint)
	
Conventions:
	latitude: decimal degree
	longitude: decimal degree
	altitude: meters
	orientation: [pitch, roll, yaw]
	
	pitch: +ve->pitch up (backward)  -ve->pitch down (forward)  
	roll: +ve->right  -ve->left
	yaw: North->0  East->pi/2  South->pi  West->3/2pi
	throttle: [0, 1024] (hover throttle: 509)
	
	rcPitch -> pitch: [1000, 2000] -> [-0.1745rad, 0.1745rad]
	rcRoll -> roll: [1000, 2000] -> [-0.1745rad, 0.1745rad] 
	rcYaw -> yaw: [1000, 2000]->[0, 2pi]
	rcThrottle -> throttle: [1000, 2000] -> [0, 1024]

	initial pose: latitude->19.0  longitude->72.0  altitude->0.31  orientation->[0, 0, 3/2pi]
'''

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from std_msgs.msg import Float32
from utils import utils
import rospy
import time
import tf
from math import sin, cos, atan2, pi, sqrt
from numpy import sign

class edrone_position_controller():
	def __init__(self):
		rospy.init_node('position_controller', anonymous=True)

		self.waypoints = []

		self.drone_gps_location = [0.0, 0.0, 0.0] #[latitude, longitude, altitude]
		self.drone_xyz = [0.0, 0.0, 0.0] #[x, y, z]
		self.drone_orientation_euler = [0.0, 0.0, 0.0] #[pitch, roll, yaw]
		self.drone_velocity = [0.0, 0.0, 0.0] #[x, y, z]
		self.velocity_mod = 0.0 #net xy velocity
		self.MAX_VELOCITY = 10
		self.velocity_flag = False
		self.obstacle_flag = False
		self.goal_flag = False
		self.bearing = 0.0 #w.r.t. North (N=0deg   E=90deg   S=180deg   W=270deg)

		self.setpoint_position_cmd = [19.0, 72.0, 5, 1750] #[latitude, longitude, altitude, yaw]
		self.setpoint_xyz = [0.0, 0.0, 0.0] #[x, y, z]
		self.error = [0.0, 0.0, 0.0] #[lt_err, lg_err, alt_err]
		self.proj_error = [0.0, 0.0, 0.0] #[pitch_err, roll_err, alt_err]
		self.prev_error = [0.0, 0.0, 0.0] #[pitch_err, roll_err, alt_err]	

		self.sample_time = 2*0.0166

		self.Kp = [11000000, 11000000, 65]
		self.Ki = [0, 0, 0.0]
		self.Kd = [35000000, 35000000, 83]
		self.proportional = [0.0, 0.0, 0.0]
		self.integral = [0.0, 0.0, 0.0]
		self.derivative = [0.0, 0.0, 0.0]

		self.pr_set_cmd = [0.0, 0.0] #[set_pitch, set_roll]
		self.throttle_cmd = 0.0 #[set_throttle]
		
		# subscribers
		rospy.Subscriber('/edrone_controller', edrone_pos_cmd, self.controller_callback)
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/edrone/gps_velocity', Vector3Stamped, self.gps_velocity_callback)
		rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
		rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
		# rospy.Subscriber('/pid_tuning', PidTune, self.any_set_pid)

		# publishers
		self.setpoint_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
		self.setpoint_msg = edrone_cmd() 

	# ----------# callbacks
	def controller_callback(self, msg):
		self.setpoint_position_cmd[0] = msg.latitude
		self.setpoint_position_cmd[1] = msg.longitude
		self.setpoint_position_cmd[2] = msg.altitude
		self.setpoint_position_cmd[3] = msg.rcYaw

		self.setpoint_xyz[0] = utils.lat_to_x(msg.latitude)
		self.setpoint_xyz[1] = utils.lon_to_y(msg.longitude)
		self.setpoint_xyz[2] = msg.altitude

	def gps_callback(self, msg):
		self.drone_gps_location[0] = msg.latitude
		self.drone_gps_location[1] = msg.longitude
		self.drone_gps_location[2] = msg.altitude

		self.drone_xyz[0] = utils.lat_to_x(msg.latitude)
		self.drone_xyz[1] = utils.lon_to_y(msg.longitude)
		self.drone_xyz[2] = msg.altitude

		distance_from_goal = sqrt((self.drone_xyz[0]-self.setpoint_xyz[0])**2 + (self.drone_xyz[1]-self.setpoint_xyz[1])**2)

		if(distance_from_goal<30):
			self.goal_flag = True
		else:
			self.goal_flag = False

	def gps_velocity_callback(self, msg):
		self.drone_velocity[0] = msg.vector.x
		self.drone_velocity[1] = msg.vector.y
		self.drone_velocity[2] = msg.vector.z
		self.velocity_mod = sqrt(self.drone_velocity[0]**2 + self.drone_velocity[1]**2)

		# print(self.is_goal_in_velocity_direction(), self.velocity_mod)

		if(self.is_goal_in_velocity_direction() and (self.velocity_mod>=self.MAX_VELOCITY)):
			self.velocity_flag = True
		else:
			self.velocity_flag = False
	
	def imu_callback(self, msg):
		(self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
		self.bearing = self.imuYaw2bearing(self.drone_orientation_euler[2])
	
	def range_finder_top_callback(self, msg):
		laser_distances = msg.ranges
		flag = False
		obstacle_direction = None

		if(laser_distances[4]>0.3 and laser_distances[4]<25):
			obstacle_direction = self.bearing
			flag = True
		if(laser_distances[3]>0.3 and laser_distances[3]<25):
			obstacle_direction = (self.bearing - pi/2.0) % (2*pi)
			flag = True
		if(laser_distances[2]>0.3 and laser_distances[2]<25):
			obstacle_direction = (self.bearing - pi) % (2*pi)
			flag = True
		if(laser_distances[1]>0.3 and laser_distances[1]<25):
			obstacle_direction = (self.bearing - 3/2.0*pi) % (2*pi)
			flag = True

		if(flag and abs(utils.angle_between_two_directions(obstacle_direction, 2*pi - utils.angle_between_two_locations(self.drone_xyz, self.setpoint_xyz)))<=pi/2.0):
			self.obstacle_flag = True
		else:
			self.obstacle_flag = False
		
		print(flag, obstacle_direction, self.obstacle_flag)

	# def any_set_pid(self, msg):
	# 	self.Kp[0] = msg.Kp*1000000
	# 	self.Ki[0] = msg.Ki*0.01
	# 	self.Kd[0] = msg.Kd*1000000

	# ----------# utility
	def imuYaw2bearing(self, imuYaw):
		return (2*pi - imuYaw - pi/2) % (2*pi)
	
	def is_goal_in_velocity_direction(self):
		vel_bearing = 2*pi - (atan2(self.drone_velocity[1], self.drone_velocity[0])%(2*pi))
		goal_bearing = 2*pi - utils.angle_between_two_locations(self.drone_xyz, self.setpoint_xyz)
		
		return (abs(utils.angle_between_two_directions(vel_bearing, goal_bearing)) < pi/16)
	
	
	# ----------# pid
	def position_pid(self):
		#Error
		self.error[0] = self.setpoint_position_cmd[0] - self.drone_gps_location[0]
		self.error[1] = self.setpoint_position_cmd[1] - self.drone_gps_location[1]
		self.error[2] = self.setpoint_position_cmd[2] - self.drone_gps_location[2]

		#Transforming error from environtment frame to drone frame
		self.proj_error[0] = -(self.error[0]*cos(self.bearing) + self.error[1]*sin(self.bearing))
		self.proj_error[1] = (-self.error[0]*sin(self.bearing) + self.error[1]*cos(self.bearing))
		self.proj_error[2] = self.setpoint_position_cmd[2] - self.drone_gps_location[2]

		#PID In
		self.proportional[0] = self.proj_error[0]
		self.integral[0] += self.proj_error[0]*self.sample_time
		self.derivative[0] = (self.proj_error[0]-self.prev_error[0])/self.sample_time
		
		self.proportional[1] = self.proj_error[1]
		self.integral[1] += self.proj_error[1]*self.sample_time
		self.derivative[1] = (self.proj_error[1]-self.prev_error[1])/self.sample_time
		
		self.proportional[2] = self.proj_error[2]
		self.integral[2] += self.proj_error[2]*self.sample_time
		self.derivative[2] = (self.proj_error[2]-self.prev_error[2])/self.sample_time

		#PID Out
		self.pr_set_cmd[0] = self.Kp[0]*self.proportional[0] + self.Ki[0]*self.integral[0] + self.Kd[0]*self.derivative[0]
		self.pr_set_cmd[1] = self.Kp[1]*self.proportional[1] + self.Ki[1]*self.integral[1] + self.Kd[1]*self.derivative[1]
		self.throttle_cmd = self.Kp[2]*self.proportional[2] + self.Ki[2]*self.integral[2] + self.Kd[2]*self.derivative[2]
		self.prev_error = self.proj_error[:]

		ratio = self.pr_set_cmd[0]/(self.pr_set_cmd[1] + 10e-7)

		if(abs(self.pr_set_cmd[0])>=500 or abs(self.pr_set_cmd[1])>=500):
			if(abs(ratio) > 1):
				if(self.pr_set_cmd[0] > 500):
					self.pr_set_cmd[0] = 500
				elif(self.pr_set_cmd[0] < -500):
					self.pr_set_cmd[0] = -500
				
				if(self.velocity_flag):
					self.pr_set_cmd[0] = 0 #self.pr_set_cmd[0]/20.0
				
				if(self.goal_flag and self.velocity_mod>4 and self.is_goal_in_velocity_direction()):
					self.pr_set_cmd[0] = -sign(self.proportional[0])*500

				if(self.obstacle_flag and self.velocity_mod>3 and self.is_goal_in_velocity_direction()):
					self.pr_set_cmd[0] = -sign(self.proportional[0])*500

				self.pr_set_cmd[1] = self.pr_set_cmd[0]/ratio
			else:
				if(self.pr_set_cmd[1] > 500):
					self.pr_set_cmd[1] = 500
				elif(self.pr_set_cmd[1] < -500):
					self.pr_set_cmd[1] = -500

				if(self.velocity_flag):
					self.pr_set_cmd[1] = 0 #self.pr_set_cmd[1]/20.0
				
				if(self.goal_flag and self.velocity_mod>4 and self.is_goal_in_velocity_direction()):
					self.pr_set_cmd[1] = -sign(self.proportional[1])*500		
				
				if(self.obstacle_flag and self.velocity_mod>3 and self.is_goal_in_velocity_direction()):
					self.pr_set_cmd[1] = -sign(self.proportional[1])*500

				self.pr_set_cmd[0] = self.pr_set_cmd[1]*ratio

		#Publish
		self.setpoint_msg.rcRoll = 1500 + self.pr_set_cmd[1]
		self.setpoint_msg.rcPitch = 1500 + self.pr_set_cmd[0]
		self.setpoint_msg.rcYaw = self.setpoint_position_cmd[3]
		self.setpoint_msg.rcThrottle = ((512 + self.throttle_cmd)- 0)/1024.0*1000 + 1000.0

		if self.setpoint_msg.rcThrottle > 2000:
			self.setpoint_msg.rcThrottle = 2000
		elif self.setpoint_msg.rcThrottle < 1000:
			self.setpoint_msg.rcThrottle = 1000
		
		self.setpoint_pub.publish(self.setpoint_msg)

if __name__ == '__main__':
	position_controller = edrone_position_controller()

	r = rospy.Rate(1/position_controller.sample_time)
	
	rospy.loginfo("Start Position Controller.")

	while not rospy.is_shutdown():	
		position_controller.position_pid()
		r.sleep()
