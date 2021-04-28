#!/usr/bin/env python
'''
Publishers:
	/edrone_controller 
		msg type: vitarana_drone/edrone_pos_cmd (my custom message for setting position setpoint)
	
Subcribers:
	/edrone/gps
		msg type: sensor_msgs/NavSatFix
	/edrone/imu/data
		msg type: sensor_msgs/Imu
	/edrone/range_finder_bottom
		msg type: sensor_msgs/LaserScan
	/edrone/range_finder_top
		msg type: sensor_msgs/LaserScan

Services:
	/edrone/activate_gripper
		srv format: vitarana_drone/Gripper
	/edrone/capture_qr
		srv format: vitarana_drone/goal_gps (my custom service for capture QR info)
	
Conventions:
	latitude: decimal degree
	longitude: decimal degree
	altitude: meters
	x: meters
	y: meters
	z: meters
	orientation: [pitch, roll, yaw]
	
	yaw/bearing: North->0  East->pi/2  South->pi  West->3/2pi
	angle in cartesian system: +ve x->0  +ve y->pi/2  -ve x->pi  -ve y->3/2pi

	bearing -> cartesian angle: cartesian angle = 2pi - bearing
	cartesian angle -> bearing: bearing = 2pi - cartesian angle
	
	rcYaw -> yaw: [1000, 2000]->[0, 2pi]

	initial pose: latitude->19.0009248718   longitude->71.9998318945   altitude->0.31  orientation->[0, 0, 3/2pi]

Notes:
	1. There are frequent conversions from bearing to cartesian angle for working in xy axes system.
	   As cartesian angle moves in opposite direction to bearing, subtracting anyone from 2pi, converts to 
	   other.
	
	2. Bug 0 algorithm has been implemented for path planning.
'''

from vitarana_drone.msg import *
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest, goal_gps, goal_gpsResponse, goal_gpsRequest
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from std_msgs.msg import Float32
import rospy
import time
import tf
from math import sin, cos, atan2, pi, sqrt

# UTILS CLASS FOR BASIC FUNCTIONS
class utils():
	@staticmethod
	def lat_to_x(lat):
		return 110692.0702932625 * (lat - 19)
	
	@staticmethod
	def lon_to_y(lon):
		return -105292.0089353767 * (lon - 72)

	@staticmethod
	def x_to_lat(x):
		return (x/110692.0702932625) + 19

	@staticmethod
	def y_to_lon(y):
		return (y/-105292.0089353767) + 72
	
	@staticmethod
	def angle_between_two_locations(loc_i, loc_f):
		return (2*pi + atan2(loc_f[1]-loc_i[1], loc_f[0]-loc_i[0])) % (2*pi)

	@staticmethod
	def distance_between_two_locations(loc_i, loc_f):
		return sqrt((loc_f[0]-loc_i[0])**2 + (loc_f[1]-loc_i[1])**2)

	@staticmethod
	def polar_to_cartesian(loc, angle, distance):
		x = loc[0] + distance*cos(angle)
		y = loc[1] + distance*sin(angle)
		return [x,y]

	@staticmethod
	def bearing_to_setpoint_yaw(bearing):
		return (bearing - 0.0)*(1000)/(2*pi) + 1000.0
	
	@staticmethod
	def imuYaw_to_bearing(imuYaw):
		return (2*pi - imuYaw - pi/2.0) % (2*pi)

		
# PLANNER CLASS
class edrone_planner():
	def __init__(self):
		rospy.init_node('plan_controller', anonymous=True)
		
		self.init_props()
		self.init_flags()

		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
		rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.laser_bottom_callback)
		rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.laser_callback)

		self.plan_setpoint_pub = rospy.Publisher('/edrone_controller', edrone_pos_cmd, queue_size=1)
	
	#--------------------------------#
	## initializations
	def init_props(self):
		self.sense_rate = 10.0
		
		self.plan_cmd_msg = edrone_pos_cmd()

		self.drone_gps_location = [0.0, 0.0, 0.0] #[latitude, longitude, altitude]
		self.drone_xyz = [0.0, 0.0, 0.0] #[x, y, z]
		self.rel_altitude = 0.0 
		self.laser_distances = [float('Inf'), float('Inf'), float('Inf'), float('Inf'), float('Inf')]
		self.drone_orientation_euler = [0.0, 0.0, 0.0] #[pitch, roll, yaw]

		self.set_heading = 0.0 # drone movement direction
		self.obstacle_direction = 0.0 # bearing of detected obstacle
		self.route_altitude = 27.0 # maintain route altitude

		# gps co-ordinates
		self.start_gps = [0.0, 0.0, 0.0]
		self.package_gps = [0.0, 0.0, 0.0]
		self.goal_gps = [0.0, 0.0, 0.0]
		# respective catersian co-ordinates
		self.start_xyz = [0.0, 0.0, 0.0]
		self.package_xyz = [0.0, 0.0, 0.0]
		self.goal_xyz = [0.0, 0.0, 0.0]

	def init_flags(self):
		self.idle_flag = True
		self.takeoff_flag = False
		self.landing_flag = False
		self.pickup_flag = False
		self.enroute_flag = False
		self.landing_on_package_flag = False
		self.acquiring_package_flag = False
		self.deacquiring_package_flag = False
		self.package_acquired_flag = False
		self.package_delivered_flag = False
		self.QR_detect_flag = False
		self.obstacle_flag = False
	
	#--------------------------------#
	## subscription callbacks
	def gps_callback(self, msg):
		self.drone_gps_location[0] = msg.latitude
		self.drone_xyz[0] = utils.lat_to_x(msg.latitude)

		self.drone_gps_location[1] = msg.longitude
		self.drone_xyz[1] = utils.lon_to_y(msg.longitude)

		self.drone_gps_location[2] = msg.altitude
		self.drone_xyz[2] = msg.altitude

	def imu_callback(self, msg):
		(self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
		self.drone_orientation_euler[2] = utils.imuYaw_to_bearing(self.drone_orientation_euler[2])
	
	def laser_bottom_callback(self, msg):
		self.rel_altitude = msg.ranges[0]
	
	def laser_callback(self, msg):
		self. laser_distances = msg.ranges

	#--------------------------------#
	## setters
	def set_start_location(self, latitude, longitude, altitude):
		self.start_gps = [latitude, longitude, altitude]
		self.start_xyz = [utils.lat_to_x(latitude), utils.lon_to_y(longitude), altitude]
	
	def set_package_location(self, latitude, longitude, altitude):
		self.package_gps = [latitude, longitude, altitude]
		self.package_xyz = [utils.lat_to_x(latitude), utils.lon_to_y(longitude), altitude]

	def set_goal_location(self, latitude, longitude, altitude):
		self.goal_gps = [latitude, longitude, altitude]
		self.goal_xyz = [utils.lat_to_x(latitude), utils.lon_to_y(longitude), altitude]

	#--------------------------------#
	def calcuate_direction_bearing(self, loc_i, loc_f):
		return (2*pi - utils.angle_between_two_locations(loc_i, loc_f))

	def in_bearing_error(self, bearing1, bearing2):
		if(abs(bearing1 - bearing2)<=0.0872665):
			return True
		return False

	#--------------------------------#
	def path_threshold_box(self):
		la = False
		lo = False
		alt = False
		
		if abs(self.plan_cmd_msg.latitude - self.drone_gps_location[0]) < 0.0000361362832:
			la = True
		if abs(self.plan_cmd_msg.longitude - self.drone_gps_location[1]) < 0.00003798958763:
			lo = True
		if abs(self.plan_cmd_msg.altitude - self.drone_gps_location[2]) < 0.1:
			alt = True

		if la and lo and alt:
			return True
		
		return False
		
	def check_threshold_box(self):
		la = False
		lo = False
		alt = False
		
		if abs(self.plan_cmd_msg.latitude - self.drone_gps_location[0]) < 0.00000090340708:
			la = True
		if abs(self.plan_cmd_msg.longitude - self.drone_gps_location[1]) < 0.00000094973969:
			lo = True
		if abs(self.plan_cmd_msg.altitude - self.drone_gps_location[2]) < 0.1:
			alt = True

		if la and lo and alt:
			return True
		
		return False
	
	#--------------------------------#
	def go_until_obstacle(self, goal_location):
		#set next setpoint if in threshold
		if (self.path_threshold_box()):
			self.set_heading = self.calcuate_direction_bearing(self.drone_xyz, goal_location)
			next_xy = utils.polar_to_cartesian(self.drone_xyz, 2*pi-self.set_heading, 9)
			self.plan_cmd_msg.latitude = utils.x_to_lat(next_xy[0])
			self.plan_cmd_msg.longitude = utils.y_to_lon(next_xy[1])
			self.plan_cmd_msg.altitude = self.route_altitude
			self.plan_cmd_msg.rcYaw = 1500

	def follow_wall(self):
		# find which laser has detected obstacle
		if(self.in_bearing_error(self.obstacle_direction, self.drone_orientation_euler[2])):
			laser_index = 4
		elif(self.in_bearing_error(self.obstacle_direction, (self.drone_orientation_euler[2]- pi/2.0) % (2*pi))):
			laser_index = 3
		elif(self.in_bearing_error(self.obstacle_direction, (self.drone_orientation_euler[2]- pi) % (2*pi))):
			laser_index = 2
		elif(self.in_bearing_error(self.obstacle_direction, (self.drone_orientation_euler[2]- 3/2.0*pi) % (2*pi))):
			laser_index = 1
		else:
			laser_index = -1
		
		# avoid obstacle
		if((not laser_index == -1) and self.laser_distances[laser_index] != float('Inf')): # safe guard from gazebo crash
			next_xy = utils.polar_to_cartesian(self.drone_xyz, 2*pi-self.obstacle_direction, self.laser_distances[laser_index]-5)
			next_xy = utils.polar_to_cartesian(next_xy, 2*pi-((self.obstacle_direction-pi/2.0)%(2*pi)), 5)
			self.plan_cmd_msg.latitude = utils.x_to_lat(next_xy[0])
			self.plan_cmd_msg.longitude = utils.y_to_lon(next_xy[1])
			self.plan_cmd_msg.altitude = self.route_altitude
			self.plan_cmd_msg.rcYaw = 1500
		
		# stop follow wall
		if(self.in_bearing_error(self.obstacle_direction, self.drone_orientation_euler[2]) and self.laser_distances[4]>8):
			self.obstacle_flag = False
		elif(self.in_bearing_error(self.obstacle_direction, (self.drone_orientation_euler[2]- pi/2.0) % (2*pi)) and self.laser_distances[3]>8):
			self.obstacle_flag = False
		elif(self.in_bearing_error(self.obstacle_direction, (self.drone_orientation_euler[2]- pi) % (2*pi)) and self.laser_distances[2]>8):
			self.obstacle_flag = False
		elif(self.in_bearing_error(self.obstacle_direction, (self.drone_orientation_euler[2]- 3/2.0*pi) % (2*pi)) and self.laser_distances[1]>8):
			self.obstacle_flag = False
		
		# safe guard from drone crashing the corners
		if(not self.obstacle_flag):
			next_xy = utils.polar_to_cartesian(self.drone_xyz, 2*pi-((self.obstacle_direction-pi/2.0)%(2*pi)), 3)
			next_xy = utils.polar_to_cartesian(next_xy, 2*pi-self.obstacle_direction, 8)
			self.plan_cmd_msg.latitude = utils.x_to_lat(next_xy[0])
			self.plan_cmd_msg.longitude = utils.y_to_lon(next_xy[1])
			self.plan_cmd_msg.altitude = self.route_altitude
			self.plan_cmd_msg.rcYaw = 1500

	def detect_obstacle(self):
		# detect obstacle if distance to collision is less than 5m
		# lower limit set to 0.3m to avoid fluctuation in reading
		if(self.laser_distances[4]>0.3 and self.laser_distances[4]<5):
			rospy.logwarn_throttle(1, "Obstacle in range [front].")
			self.obstacle_flag = True
			self.obstacle_direction = self.drone_orientation_euler[2]
		if(self.laser_distances[3]>0.3 and self.laser_distances[3]<5):
			rospy.logwarn_throttle(1, "Obstacle in range [left].")
			self.obstacle_flag = True
			self.obstacle_direction = (self.drone_orientation_euler[2] - pi/2.0) % (2*pi)
		if(self.laser_distances[2]>0.3 and self.laser_distances[2]<5):
			rospy.logwarn_throttle(1, "Obstacle in range [rear].")
			self.obstacle_flag = True
			self.obstacle_direction = (self.drone_orientation_euler[2] - pi) % (2*pi)
		if(self.laser_distances[1]>0.3 and self.laser_distances[1]<5):
			rospy.logwarn_throttle(1, "Obstacle in range [right].")
			self.obstacle_flag = True
			self.obstacle_direction = (self.drone_orientation_euler[2] - 3/2.0*pi) % (2*pi)
		
		# check if the obstacle is relevant wrt to goal
		if(self.obstacle_flag):
			if(self.pickup_flag and abs(self.obstacle_direction-self.calcuate_direction_bearing(self.drone_xyz, self.package_xyz))>pi/2.0):
				self.obstacle_flag = False
			elif(self.enroute_flag and abs(self.obstacle_direction-self.calcuate_direction_bearing(self.drone_xyz, self.goal_xyz))>pi/2.0):
				self.obstacle_flag = False

	#--------------------------------#
	def takeoff(self):
		self.plan_cmd_msg.latitude = self.drone_gps_location[0]
		self.plan_cmd_msg.longitude = self.drone_gps_location[1]
		self.plan_cmd_msg.altitude = self.route_altitude
		self.plan_cmd_msg.rcYaw = 1500
		
		if(self.check_threshold_box()):
			self.takeoff_flag = False
			rospy.loginfo("Takeoff procedure done.")
			if(self.package_acquired_flag):
				self.enroute_flag = True
			else:
				self.pickup_flag = True

	def landing(self):
		self.plan_cmd_msg.latitude = self.goal_gps[0]
		self.plan_cmd_msg.longitude = self.goal_gps[1]
		self.plan_cmd_msg.altitude = self.goal_gps[2]
		self.plan_cmd_msg.rcYaw = 1500
		
		if(self.check_threshold_box()):
			self.landing_flag = False
			rospy.loginfo("Landed.")
			if(self.package_acquired_flag):
				self.deacquiring_package_flag = True
			else:
				self.idle_flag = True

	def pickup(self):
		rospy.loginfo_once('On way to fetch package.')

		if(self.obstacle_flag):
			self.follow_wall()
		else:
			if(utils.distance_between_two_locations(self.drone_xyz, self.package_xyz)<=5):
				self.plan_cmd_msg.latitude = self.package_gps[0]
				self.plan_cmd_msg.longitude = self.package_gps[1]
				self.plan_cmd_msg.altitude = self.route_altitude
				self.plan_cmd_msg.rcYaw = 1500

				if(self.check_threshold_box()):
					self.pickup_flag = False
					rospy.loginfo("Reached pickup location.")
					self.landing_on_package_flag = True
			elif not self.obstacle_flag:
				self.go_until_obstacle(self.package_xyz)
	
	def enroute(self):
		rospy.loginfo_once("En-route.")

		if(self.obstacle_flag):
			self.follow_wall()	
		else:
			if(utils.distance_between_two_locations(self.drone_xyz, self.goal_xyz)<=5):
				self.plan_cmd_msg.latitude = self.goal_gps[0]
				self.plan_cmd_msg.longitude = self.goal_gps[1]
				self.plan_cmd_msg.altitude = self.route_altitude
				self.plan_cmd_msg.rcYaw = 1500
				
				if(self.check_threshold_box()):
					self.enroute_flag = False
					rospy.loginfo("Reached delivery location.")
					self.landing_flag = True
			else:
				self.go_until_obstacle(self.goal_xyz)
	
	def landing_on_package(self):
		rospy.loginfo_once('Detecting QR tag.')

		self.plan_cmd_msg.latitude = self.package_gps[0]
		self.plan_cmd_msg.longitude = self.package_gps[1]
		self.plan_cmd_msg.altitude = self.drone_gps_location[2] + (1 - self.rel_altitude)
		self.plan_cmd_msg.rcYaw = 1500
		
		if(self.check_threshold_box()):
			self.landing_on_package_flag = False
			self.acquiring_package_flag = True

		if(not self.QR_detect_flag):
			self.capture_qr()

	def acquiring_package(self):
		rospy.loginfo_once('Acquiring package.')

		rospy.wait_for_service('/edrone/activate_gripper')
		try:
			activate_gripper = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
			res = activate_gripper(True)
			if res.result:
				self.acquiring_package_flag = False
				rospy.loginfo("Package acquired.")
				self.package_acquired_flag = True
				self.takeoff_flag = True
			else:
				self.plan_cmd_msg.altitude -= 0.025 #decrease altitude if package not in range
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: %s" % (e))
	
	def deacquiring_package(self):
		rospy.wait_for_service('/edrone/activate_gripper')
		try:
			activate_gripper = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
			res = activate_gripper(False)
			if res.result == False:
				self.deacquiring_package_flag = False
				self.package_acquired_flag = False
				self.QR_detect_flag = False
				self.landing_flag = False
				rospy.loginfo("Package Delivered")
				self.package_delivered_flag = True
				self.idle_flag = True
			else:
				rospy.loginfo("Error in deacquiring package.")
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: %s" % (e))
	
	def capture_qr(self):
		rospy.wait_for_service('/edrone/capture_qr')
		try:
			qr_detect = rospy.ServiceProxy('/edrone/capture_qr', goal_gps)
			req = goal_gpsRequest()
			req.capture_qr = True
			res = qr_detect(req)
			if res.success:
				self.QR_detect_flag = True
				rospy.loginfo("QR tag detected.")
				self.set_goal_location(res.latitude, res.longitude, res.altitude)
				rospy.loginfo("Goal:\nlatitude: {}\nlongitude: {}\naltitude: {}".format(\
					self.goal_gps[0], self.goal_gps[1], self.goal_gps[2]))
			else:
				rospy.logwarn_throttle(1, "QR not in range.")
		except rospy.ServiceException as e:
			rospy.logerr("Service call failed: %s" % (e))

	#--------------------------------#
	#--------------------------------#
	## main control loop
	def loop(self):
		self.detect_obstacle()

		if(self.takeoff_flag):
			self.takeoff()
		
		if(self.landing_flag):
			self.landing()
		
		if(self.pickup_flag):
			self.pickup()

		if(self.enroute_flag):
			self.enroute()
		
		if(self.landing_on_package_flag):
			self.landing_on_package()
		
		if(self.acquiring_package_flag):
			self.acquiring_package()
		
		if(self.deacquiring_package_flag):
			self.deacquiring_package()
		
		self.plan_setpoint_pub.publish(self.plan_cmd_msg)
		
if __name__ == '__main__':
	planner = edrone_planner()
	r = rospy.Rate(planner.sense_rate)
	
	planner.set_start_location(19.0009248718, 71.9998318945, 22.16)
	planner.set_package_location(19.0007046575, 71.9998955286, 22.1599967919)

	planner.idle_flag = False
	rospy.loginfo("Start Navigation.")
	planner.takeoff_flag = True

	while not rospy.is_shutdown():
		planner.loop()
		r.sleep()