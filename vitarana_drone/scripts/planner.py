#!/usr/bin/env python

'''
Publishers:
	/edrone_controller 
		msg type: vitarana_drone/edrone_pos_cmd (my custom message for setting position setpoint)
	/edrone/marker_data
		msg type: vitarana_drone/MarkerData
	
Subcribers:
	/edrone/gps
		msg type: sensor_msgs/NavSatFix
	/edrone/imu/data
		msg type: sensor_msgs/Imu
	/edrone/range_finder_bottom
		msg type: sensor_msgs/LaserScan
	/edrone/range_finder_top
		msg type: sensor_msgs/LaserScan
	/edrone/marker_data_raw
		msg type: vitarana_drone/MarkerData

Services:
	/edrone/activate_gripper
		srv format: vitarana_drone/Gripper
	
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

States:
-IDLE
-PLAN
-TAKEOFF
-EN_ROUTE
	-OBSTACLE_DETECTION
	-GO_UNTIL_OBSTACLE
	-FOLLOW_WALL
	-FIND_WALL
-SEARCH
	-SENSE
	-FIND_MARKER
	-GO_TO_MAKRER
-CHANGE_ALTITUDE
-PACKAGE_RETRIEVAL
	-PACKAGE_ACQUIRE
-PACKAGE_DELIVERY
	-PACKAGE_DEACQUIRE
-LAND

Notes:
	1. There are frequent conversions from bearing to cartesian angle for working in xy axes system.
	   As cartesian angle moves in opposite direction to bearing, subtracting anyone from 2pi, converts to 
	   other.
	
	2. Bug 0 algorithm has been implemented for path planning.
'''

from vitarana_drone.msg import *
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest, goal_gps, goal_gpsResponse, goal_gpsRequest
from utils import utils
from warehouse import Warehouse
from optimizer2 import *
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from math import sin, cos, tan, atan2, pi, sqrt, isnan
import copy
import rospy
import tf
import signal
from sys import exit
import time
import enum
import smach
import smach_ros

#--------------------------------#
# Signal Handler
#--------------------------------#
def signal_handler(sig, frame):
	dth.plan_cmd_msg.latitude = dth.gps_location[0]
	dth.plan_cmd_msg.longitude = dth.gps_location[1]
	dth.plan_cmd_msg.altitude = dth.gps_location[2]
	dth.plan_cmd_msg.rcYaw = utils.bearing_to_setpoint_yaw(dth.orientation_euler[2])

	rospy.loginfo('Exit.')
	
	t = time.time()
	while (time.time() - t) < 3:
		dth.plan_setpoint_pub.publish(dth.plan_cmd_msg)

	exit(0)

#--------------------------------#
# Enums
#--------------------------------#
class goalType(enum.Enum):
	SOURCE = 0
	FLY_BY = 1
	PACKAGE_RETRIEVAL = 2
	PACKAGE_DELIVERY = 3
	PACKAGE_RETURN = 4
	LAND = 5
	DESTINATION = 6

class reason(enum.Enum):
	OBSTACLE = 0
	GO_TO_GOAL_ALTITUDE = 1
	SEARCH = 2
	ROUTE_ALTITUDE = 3
	QR_DETECT = 4
	PACKAGE_DELIVERY = 5

class land(enum.Enum):
	SEARCH = 0
	EXACT = 1

#--------------------------------#
# Drone Topic Handler
#--------------------------------#
class DroneTopicHandler():
	def __init__(self):
		self.init_props()
		self.init_subscribers()
		self.init_publishers()
	
	def init_props(self):
		self.gps_location = [0.0, 0.0, 0.0]
		self.xyz = [0.0, 0.0, 0.0]
		self.orientation_euler = [0.0, 0.0, 0.0]
		self.rel_altitude = 0.0
		self.laser_distances = [float('Inf'), float('Inf'), float('Inf'), float('Inf'), float('Inf')]
		self.err_xy_m = [float('nan'), float('nan')]
		self.is_marker_in_scene = False

		self.plan_cmd_msg = edrone_pos_cmd()
		self.marker_data_msg = MarkerData()
		self.marker_data_msg.marker_id = -1
	
	def init_subscribers(self):
		rospy.Subscriber('/edrone/gps', NavSatFix, self._gps_callback)
		rospy.Subscriber('/edrone/imu/data', Imu, self._imu_callback)
		rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self._laser_bottom_callback)
		rospy.Subscriber('/edrone/range_finder_top', LaserScan, self._laser_callback)
		rospy.Subscriber('/edrone/marker_data_raw', MarkerData, self._marker_data_raw_callback)
	
	def init_publishers(self):
		self.plan_setpoint_pub = rospy.Publisher('/edrone_controller', edrone_pos_cmd, queue_size=1)
		self.marker_data_pub = rospy.Publisher('/edrone/marker_data', MarkerData, queue_size=1)
	
	def _gps_callback(self, msg):
		self.gps_location[0] = msg.latitude
		self.xyz[0] = utils.lat_to_x(msg.latitude)
		self.gps_location[1] = msg.longitude
		self.xyz[1] = utils.lon_to_y(msg.longitude)
		self.gps_location[2] = msg.altitude
		self.xyz[2] = msg.altitude
	
	def _imu_callback(self, msg):
		(self.orientation_euler[0], self.orientation_euler[1], self.orientation_euler[2]) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
		self.orientation_euler[2] = utils.imuYaw_to_bearing(self.orientation_euler[2])
	
	def _laser_bottom_callback(self, msg):
		if(msg.ranges[0]>0.4):
			self.rel_altitude = msg.ranges[0]

	def _laser_callback(self, msg):
		self.laser_distances = msg.ranges

	def _marker_data_raw_callback(self, msg):
		self.err_xy_m[0] = msg.err_x_m
		self.err_xy_m[1] = msg.err_y_m
		self.marker_data_msg.err_x_m = msg.err_x_m
		self.marker_data_msg.err_y_m = msg.err_y_m

		if(not isnan(msg.err_x_m)):
			self.is_marker_in_scene = True
		else:
			self.is_marker_in_scene = False
			
		self.marker_data_pub.publish(self.marker_data_msg)

	def set_plan_cmd_msg(self, latitude, longitude, altitude, rcYaw):
		self.plan_cmd_msg.latitude = latitude
		self.plan_cmd_msg.longitude = longitude
		self.plan_cmd_msg.altitude = altitude
		self.plan_cmd_msg.rcYaw = rcYaw

	def set_marker_data_msg(self, marker_id, err_x_m, err_y_m):
		self.marker_data_msg.marker_id = marker_id
		self.marker_data_msg.err_x_m = err_x_m
		self.marker_data_msg.err_y_m = err_y_m

	def publish_plan_cmd_msg(self, sleep=0.1):
		self.plan_setpoint_pub.publish(self.plan_cmd_msg)
		time.sleep(sleep)

	def publish_marker_data_msg(self, sleep=0.1):
		self.marker_data_pub.publish(self.marker_data_msg)
		time.sleep(sleep)

#--------------------------------#
# Flight Plan
#--------------------------------#
class FlightPlan():
	def __init__(self):
		self.init_props()
		self.init_ui_props()
	
	def init_props(self):
		self.source_gps = [float('nan'), float('nan'), float('nan')]
		self.source_xyz = [float('nan'), float('nan'), float('nan')]

		self.destination_gps = [float('nan'), float('nan'), float('nan')]
		self.destination_xyz = [float('nan'), float('nan'), float('nan')]
		
		self.plan_gps = list()
		self.plan_xyz = list()

		self.current = 0
		self.activated = False
		self.destination_reached = False

	def init_ui_props(self):
		self.prompt = "[flight plan] -> "
	
	def set_source(self, gps, name):
		self.source_gps = gps
		self.source_xyz = (utils.lat_to_x(gps[0]), utils.lon_to_y(gps[1]), gps[2])
		
		self.insert(self.source_gps, goalType.SOURCE, name)

	def set_destination(self, gps, name):
		self.destination_gps = gps
		self.destination_xyz = (utils.lat_to_x(gps[0]), utils.lon_to_y(gps[1]), gps[2])

		self.insert(self.destination_gps, goalType.DESTINATION, name)

	def is_source_set(self):
		if(isnan(self.source_gps[0])):
			return False
		else:
			return True
	
	def is_destination_set(self):
		if(isnan(self.destination_gps[0])):
			return False
		else:
			return True

	def get_current_goal(self, format='gps'):
		if(format == 'gps'):
			return self.plan_gps[self.current]
		elif(format == 'xyz'):
			return self.plan_xyz[self.current]

	def get_next_goal(self, format='gps'):
		if(self.current+1 < len(self.plan_gps)):
			if(format == 'gps'):
				return self.plan_gps[self.current+1]
			elif(format == 'xyz'):
				return self.plan_xyz[self.current+1]
	
	def get_prev_goal(self, format='gps'):
		if(self.current-1 >= 0):
			if(format == 'gps'):
				return self.plan_gps[self.current-1]
			elif(format == 'xyz'):
				return self.plan_xyz[self.current-1]

	def jump_to_next_goal(self):
		self.current += 1
		goal_name = self.get_current_goal()[2]
		return goal_name

	def insert(self, gps, type, name):
		if(type == goalType.SOURCE):
			self.plan_gps.insert(0, (gps, goalType.SOURCE, name))
		elif(type == goalType.FLY_BY):
			if(self.is_destination_set()):
				self.plan_gps.insert(len(self.plan_gps)-1, (gps, goalType.FLY_BY, name))
			else:
				self.plan_gps.append((gps, goalType.FLY_BY, name))
		elif(type == goalType.PACKAGE_RETRIEVAL):
			if(self.is_destination_set()):
				self.plan_gps.insert(len(self.plan_gps)-1, (gps, goalType.PACKAGE_RETRIEVAL, name))
			else:
				self.plan_gps.append((gps, goalType.PACKAGE_RETRIEVAL, name))
		elif(type == goalType.PACKAGE_DELIVERY):
			if(self.is_destination_set()):
				self.plan_gps.insert(len(self.plan_gps)-1, (gps, goalType.PACKAGE_DELIVERY, name))
			else:
				self.plan_gps.append((gps, goalType.PACKAGE_DELIVERY, name))
		elif(type == goalType.PACKAGE_RETURN):
			if(self.is_destination_set()):
				self.plan_gps.insert(len(self.plan_gps)-1, (gps, goalType.PACKAGE_RETURN, name))
			else:
				self.plan_gps.append((gps, goalType.PACKAGE_RETURN, name))
		elif(type == goalType.LAND):
			if(self.is_destination_set()):
				self.plan_gps.insert(len(self.plan_gps)-1, (gps, goalType.LAND, name))
			else:
				self.plan_gps.append((gps, goalType.LAND, name))
		elif(type == goalType.DESTINATION):
			self.plan_gps.append((gps, goalType.DESTINATION, name))

	def activate(self):
		if(not self.is_destination_set()):
			goal = self.plan_gps.pop()
			self.set_destination(goal[0])	

		for goal in self.plan_gps:
			self.plan_xyz.append(((utils.lat_to_x(goal[0][0]), utils.lon_to_y(goal[0][1]), goal[0][2]), goal[1], goal[2]))
		
		self.activated = True
	
	def summary(self):
		result = "{}".format(self.prompt)
		result += "Flight Plan :\n\n"
		result += "Source : "
		result += "(lat: {}, lon: {}, alt: {})\n".format(self.source_gps[0], self.source_gps[1], self.source_gps[2])
		result += "Destination :"
		result += "(lat: {}, lon: {}, alt: {})\n".format(self.destination_gps[0], self.destination_gps[1], self.destination_gps[2])
		result += "\n"
		result += "Route :\n"
		for goal in self.plan_gps:
			result += "{} - (lat: {}, lon: {}, alt: {}) -> ".format(goal[2], goal[0][0], goal[0][1], goal[0][2])
			if goal[1] == goalType.SOURCE:
				result += "SOURCE\n"
			elif goal[1] == goalType.FLY_BY:
				result += "FLY_BY\n"
			elif goal[1] == goalType.PACKAGE_RETRIEVAL:
				result += "PACKAGE_RETRIEVAL\n"
			elif goal[1] == goalType.PACKAGE_DELIVERY:
				result += "PACKAGE_DELIVERY\n"
			elif goal[1] == goalType.PACKAGE_RETURN:
				result += "PACKAGE_RETURN\n"
			elif goal[1] == goalType.LAND:
				result += "LAND\n"
			elif goal[1] == goalType.DESTINATION:
				result += "DESTINATION\n"
		
		print(result)
		
	def run(self):
		self.summary()
		print("")
		print("{}Press enter to start navigation.".format(self.prompt))
		raw_input("{}".format(self.prompt))

	def __str__(self):
		result = ""
		result += "Flight Plan :\n\n"
		result += "Source : "
		result += "(lat: {}, lon: {}, alt: {})\n".format(self.source_gps[0], self.source_gps[1], self.source_gps[2])
		result += "Destination :"
		result += "(lat: {}, lon: {}, alt: {})\n".format(self.destination_gps[0], self.destination_gps[1], self.destination_gps[2])
		result += "\n"
		result += "Route :\n"
		for goal in self.plan_gps:
			result += "{} - (lat: {}, lon: {}, alt: {}) -> ".format(goal[2], goal[0][0], goal[0][1], goal[0][2])
			if goal[1] == goalType.SOURCE:
				result += "SOURCE\n"
			elif goal[1] == goalType.FLY_BY:
				result += "FLY_BY\n"
			elif goal[1] == goalType.PACKAGE_RETRIEVAL:
				result += "PACKAGE_RETRIEVAL\n"
			elif goal[1] == goalType.PACKAGE_DELIVERY:
				result += "PACKAGE_DELIVERY\n"
			elif goal[1] == goalType.PACKAGE_RETURN:
				result += "PACKAGE_RETURN\n"
			elif goal[1] == goalType.LAND:
				result += "LAND\n"
			elif goal[1] == goalType.DESTINATION:
				result += "DESTINATION\n"
		
		return result

#--------------------------------#
# Helper Functions
#--------------------------------#
def check_threshold_box():
	la = False
	lo = False
	alt = False
	
	if abs(dth.plan_cmd_msg.latitude - dth.gps_location[0]) < 0.000004517:
		la = True
	if abs(dth.plan_cmd_msg.longitude - dth.gps_location[1]) < 0.0000047487:
		lo = True
	if abs(dth.plan_cmd_msg.altitude - dth.gps_location[2]) < 0.2:
		alt = True

	if la and lo and alt:
		return True
	
	return False

def path_threshold_box():
	la = False
	lo = False
	alt = False
	
	if abs(dth.plan_cmd_msg.latitude - dth.gps_location[0]) < 0.0000361362832:
		la = True
	if abs(dth.plan_cmd_msg.longitude - dth.gps_location[1]) < 0.00003798958763:
		lo = True
	if abs(dth.plan_cmd_msg.altitude - dth.gps_location[2]) < 0.1:
		alt = True

	if la and lo and alt:
		return True
	
	return False

def calcuate_direction_bearing(loc_i, loc_f):
	return (2*pi - utils.angle_between_two_locations(loc_i, loc_f))

def in_bearing_error(bearing1, bearing2):
	if(abs(utils.angle_between_two_directions(bearing1, bearing2))<=0.0872665):
		return True
	return False

#--------------------------------#
# States
#--------------------------------#
# Idle State
class Idle(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['loop', 'plan'], input_keys=['planner'], output_keys=['prev_state', 'planner'])
	
	def execute(self, ud):
		if(ud.planner.activated and (not ud.planner.destination_reached)):
			ud.prev_state = 'IDLE'
			return 'plan'
		else:			
			ud.prev_state = 'IDLE'
			return 'loop'

# Plan State
class Plan(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['idle', 'takeoff', 'change_altitude', 'en_route', 'search', 'land', 'package_retrieval', 'package_delivery'], input_keys=['prev_state', 'reason', 'planner', 'route_altitude'], output_keys=['prev_state', 'reason', 'planner', 'route_altitude', 'real_goal_gps'])

	def execute(self, ud):
		if(ud.prev_state == 'IDLE'):
			self.transition_from_idle(ud)
			ud.prev_state = 'PLAN'
			return 'takeoff'
		elif(ud.prev_state == 'TAKEOFF'):
			ud.prev_state = 'PLAN'
			return 'en_route'
		elif(ud.prev_state == 'EN_ROUTE'):
			outcome = self.transition_from_en_route(ud)
			ud.prev_state = 'PLAN'
			return outcome
		elif(ud.prev_state == 'CHANGE_ALTITUDE'):
			outcome = self.transition_from_change_altitude(ud)
			ud.prev_state = 'PLAN'
			return outcome
		elif(ud.prev_state == 'SEARCH'):
			outcome = self.transition_from_search(ud)
			ud.prev_state = 'PLAN'
			return outcome
		elif(ud.prev_state == 'LAND'):
			outcome = self.transition_from_land(ud)
			ud.prev_state = 'PLAN'
			return outcome
		elif(ud.prev_state == 'PACKAGE_RETRIEVAL'):
			self.transition_from_package_retrieval(ud)
			ud.prev_state = 'PLAN'
			return 'takeoff'
		elif(ud.prev_state == 'PACKAGE_DELIVERY'):
			self.transition_from_package_delivery(ud)
			ud.prev_state = 'PLAN'
			return 'takeoff'
	
	def set_route_altitude(self, goal_xyz):
		if(goal_xyz[2] > dth.gps_location[2]):
			return goal_xyz[2] + 15
		elif((dth.gps_location[2]-(goal_xyz[2]+10)) < dth.rel_altitude):
			return goal_xyz[2] + 15
		else:
			return dth.gps_location[2] + 10
	
	def transition_from_idle(self, ud):
		goal_name = ud.planner.jump_to_next_goal()
		dth.marker_data_msg.marker_id = int(goal_name[-1])
		goal_xyz = ud.planner.get_current_goal(format='xyz')[0]
		ud.route_altitude = self.set_route_altitude(goal_xyz)

	def transition_from_en_route(self, ud):
		goal = ud.planner.get_current_goal(format='xyz')

		if(goal[1] in [goalType.FLY_BY, goalType.LAND, goalType.PACKAGE_DELIVERY]):
			ud.reason = reason.SEARCH
			return 'change_altitude'
		elif(goal[1] in [goalType.PACKAGE_RETRIEVAL]):
			ud.reason = reason.QR_DETECT
			return 'change_altitude'
		elif(goal[1] in [goalType.PACKAGE_RETURN]):
			ud.reason = reason.PACKAGE_DELIVERY
			return 'change_altitude'
		elif(goal[1] in [goalType.DESTINATION]):
			return 'land'

	def transition_from_search(self, ud):
		goal = ud.planner.get_current_goal(format='xyz')

		if(goal[1] == goalType.FLY_BY):
			goal_name = ud.planner.jump_to_next_goal()
			dth.marker_data_msg.marker_id = int(goal_name[-1])
			goal_xyz = ud.planner.get_current_goal(format='xyz')[0]
			ud.route_altitude = self.set_route_altitude(goal_xyz)
			ud.reason = reason.ROUTE_ALTITUDE
			return 'change_altitude'
		elif(goal[1] in [goalType.LAND, goalType.DESTINATION]):
			ud.real_goal_gps = [dth.gps_location[0], dth.gps_location[1], goal[0][2]]
			return 'land'
		elif(goal[1] == goalType.PACKAGE_DELIVERY):
			ud.reason = reason.PACKAGE_DELIVERY
			return 'change_altitude'
	
	def transition_from_land(self, ud):
		goal = ud.planner.get_current_goal(format='xyz')
		if(goal[1] == goalType.DESTINATION):
			ud.planner.destination_reached = True
			return 'idle'

	def transition_from_change_altitude(self, ud):
		if(ud.reason == reason.GO_TO_GOAL_ALTITUDE):
			ud.reason = reason.SEARCH
			return 'change_altitude'
		elif(ud.reason == reason.ROUTE_ALTITUDE):
			return 'en_route'
		elif(ud.reason == reason.SEARCH):
			return 'search'
		elif(ud.reason == reason.QR_DETECT):
			return 'package_retrieval'
		elif(ud.reason == reason.PACKAGE_DELIVERY):
			return 'package_delivery'

	def transition_from_package_retrieval(self, ud):
		ud.planner.jump_to_next_goal()
		goal_xyz = ud.planner.get_current_goal(format='xyz')[0]
		ud.route_altitude = self.set_route_altitude(goal_xyz)

	def transition_from_package_delivery(self, ud):
		ud.planner.jump_to_next_goal()
		goal_xyz = ud.planner.get_current_goal(format='xyz')[0]
		ud.route_altitude = self.set_route_altitude(goal_xyz)

# Takeoff State
class Takeoff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['loop', 'plan'], input_keys=['planner', 'route_altitude'], output_keys=['prev_state', 'planner'])
	
	def execute(self, ud):
		# dth.set_plan_cmd_msg(dth.gps_location[0], dth.gps_location[1], ud.route_altitude, 1750)
		dth.plan_cmd_msg.altitude = ud.route_altitude
		dth.publish_plan_cmd_msg()
		ud.prev_state = 'TAKEOFF'
		return 'plan'

		# if(self.takeoff_threshold_box()):
		# 	ud.prev_state = 'TAKEOFF'
		# 	return 'plan'
		# else:
		# 	ud.prev_state = 'TAKEOFF'
		# 	return 'loop'
	
	def takeoff_threshold_box(self):
		return abs(dth.plan_cmd_msg.altitude - dth.gps_location[2]) < 0.2

# Obstacle Detection State
class ObstacleDetection(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['go_until_obstacle', 'follow_wall', 'find_wall'], input_keys=['prev_state', 'planner', 'route_altitude', 'obstacle_flag', 'obstacle_direction'], output_keys=['prev_state', 'planner', 'xyz', 'obstacle_flag', 'obstacle_direction'])
	
	def execute(self, ud):
		self.set_obstacle_flag(ud)
		if(ud.obstacle_flag):
			ud.obstacle_flag = self.is_obstacle_relevant(ud)
		if(ud.obstacle_flag):
			self.check_unset_obstacle_flag(ud)
		
		if(ud.obstacle_flag):
			ud.prev_state = 'OBSTACLE_DETECTION'
			return 'follow_wall'
		elif((not ud.obstacle_flag) and ud.prev_state == 'FOLLOW_WALL'):
			ud.xyz = copy.deepcopy(dth.xyz)
			ud.prev_state = 'OBSTACLE_DETECTION'
			return 'find_wall'
			# self.find_wall(ud)
			# rospy.loginfo((self.check_obstacle(ud), self.is_obstacle_relevant(ud)))
			# obstacle_flag = self.check_obstacle(ud) and self.is_obstacle_relevant(ud)
			
			if(ud.obstacle_flag):
				ud.prev_state = 'OBSTACLE_DETECTION'
				return 'follow_wall'
			else:
				ud.prev_state = 'OBSTACLE_DETECTION'
				return 'go_until_obstacle'
		else:
			ud.prev_state = 'OBSTACLE_DETECTION'
			return 'go_until_obstacle'
		
	def set_obstacle_flag(self, ud):
		if(dth.laser_distances[4]>0.3 and dth.laser_distances[4]<8):
			ud.obstacle_direction = dth.orientation_euler[2]
			ud.obstacle_flag = True
		if(dth.laser_distances[3]>0.3 and dth.laser_distances[3]<8):
			ud.obstacle_direction = (dth.orientation_euler[2] - pi/2.0) % (2*pi)
			ud.obstacle_flag = True
		if(dth.laser_distances[2]>0.3 and dth.laser_distances[2]<8):
			ud.obstacle_direction = (dth.orientation_euler[2] - pi) % (2*pi)
			ud.obstacle_flag = True
		if(dth.laser_distances[1]>0.3 and dth.laser_distances[1]<8):
			ud.obstacle_direction = (dth.orientation_euler[2] - 3/2.0*pi) % (2*pi)
			ud.obstacle_flag = True

	def check_unset_obstacle_flag(self, ud):
		if(in_bearing_error(ud.obstacle_direction, dth.orientation_euler[2]) and dth.laser_distances[4]>8):
			ud.obstacle_flag = False
		elif(in_bearing_error(ud.obstacle_direction, (dth.orientation_euler[2]- pi/2.0) % (2*pi)) and dth.laser_distances[3]>8):
			ud.obstacle_flag = False
		elif(in_bearing_error(ud.obstacle_direction, (dth.orientation_euler[2]- pi) % (2*pi)) and dth.laser_distances[2]>8):
			ud.obstacle_flag = False
		elif(in_bearing_error(ud.obstacle_direction, (dth.orientation_euler[2]- 3/2.0*pi) % (2*pi)) and dth.laser_distances[1]>8):
			ud.obstacle_flag = False

	def is_obstacle_relevant(self, ud):
		return not (abs(utils.angle_between_two_directions(ud.obstacle_direction, calcuate_direction_bearing(dth.xyz, ud.planner.get_current_goal(format='xyz')[0])))>pi/2.0)
	
	def find_wall(self, ud):
		next_xy = utils.polar_to_cartesian(dth.xyz, 2*pi-((ud.obstacle_direction-pi/2.0)%(2*pi)), 3)
		dth.set_plan_cmd_msg(utils.x_to_lat(next_xy[0]), utils.y_to_lon(next_xy[1]), ud.route_altitude, 1750)
		dth.publish_plan_cmd_msg()
		time.sleep(3)
		
		next_xy = utils.polar_to_cartesian(next_xy, 2*pi-ud.obstacle_direction, 13)
		dth.set_plan_cmd_msg(utils.x_to_lat(next_xy[0]), utils.y_to_lon(next_xy[1]), ud.route_altitude, 1750)
		dth.publish_plan_cmd_msg()
		time.sleep(5)
		
# Go Until Obstacle State
class GoUntilObstacle(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['obstacle_detection', 'location_reached'], input_keys=['planner', 'route_altitude'], output_keys=['prev_state', 'planner'])
	
	def execute(self, ud):
		goal = ud.planner.get_current_goal()
		goal_xyz = ud.planner.get_current_goal(format='xyz')[0]

		dth.set_plan_cmd_msg(goal[0][0], goal[0][1], ud.route_altitude, 1750)
		dth.publish_plan_cmd_msg()

		if(check_threshold_box() or self.is_marker_in_range(ud)):
			ud.prev_state = 'EN_ROUTE'
			return 'location_reached'
		else:
			ud.prev_state = 'GO_UNTIL_OBSTACLE'
			return 'obstacle_detection'
	
	def is_marker_in_range(self, ud):
		goal = ud.planner.get_current_goal(format='xyz')
		goal_xyz = goal[0]
		goal_type = goal[1]

		distance_from_goal = sqrt((dth.xyz[0]-goal_xyz[0])**2 + (dth.xyz[1]-goal_xyz[1])**2)
		
		if(dth.is_marker_in_scene and distance_from_goal<10 and goal_type==goalType.PACKAGE_DELIVERY):
			return True
		
		return False

# Follow Wall State
class FollowWall(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['obstacle_detection'], input_keys=['obstacle_direction', 'route_altitude'], output_keys=['prev_state'])
	
	def execute(self, ud):
		laser_index = self.find_laser_index(ud.obstacle_direction)

		if(self.laser_index_safe_to_use(laser_index)):
			next_xy = self.avoid_obstacle(laser_index, ud.obstacle_direction)
			dth.set_plan_cmd_msg(utils.x_to_lat(next_xy[0]), utils.y_to_lon(next_xy[1]), ud.route_altitude, 1750)
			dth.publish_plan_cmd_msg()
			
		ud.prev_state = 'FOLLOW_WALL'
		return 'obstacle_detection'		

	def find_laser_index(self, obstacle_direction):
		laser_index = -1
		
		if(in_bearing_error(obstacle_direction, dth.orientation_euler[2])):
			laser_index = 4
		elif(in_bearing_error(obstacle_direction, (dth.orientation_euler[2] - pi/2.0) % (2*pi))):
			laser_index = 3
		elif(in_bearing_error(obstacle_direction, (dth.orientation_euler[2] - pi) % (2*pi))):
			laser_index = 2
		elif(in_bearing_error(obstacle_direction,(dth.orientation_euler[2] - 3/2.0*pi) % (2*pi))):
			laser_index = 1

		return laser_index
	
	def laser_index_safe_to_use(self, laser_index):
		return (not laser_index==-1) and dth.laser_distances[laser_index]!=float('Inf')
	
	def avoid_obstacle(self, laser_index, obstacle_direction):
		distance_from_wall = dth.laser_distances[laser_index]
		if(distance_from_wall > 0.3):
			next_xy = utils.polar_to_cartesian(dth.xyz, 2*pi-obstacle_direction, distance_from_wall-5)
			next_xy = utils.polar_to_cartesian(next_xy, 2*pi-((obstacle_direction-pi/2.0)%(2*pi)), 6)
		else:
			next_xy = utils.polar_to_cartesian(dth.xyz, 2*pi-((obstacle_direction-pi/2.0)%(2*pi)), 6)

		return next_xy

#Find Wall State
class FindWall(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['loop', 'follow_wall', 'go_until_obstacle'], input_keys=['prev_state', 'planner', 'xyz', 'route_altitude', 'obstacle_direction'], output_keys=['prev_state', 'planner', 'obstacle_flag', 'obstacle_direction'])
	
	def execute(self, ud):
		self.go_to_next_xy(ud)
		if(self.check_obstacle(ud)):
			if(self.is_obstacle_relevant(ud)):
				ud.obstacle_flag = True
				ud.prev_state = 'FIND_WALL'
				return 'follow_wall'
			else:
				ud.prev_state = 'FIND_WALL'
				return 'go_until_obstacle'
		elif(check_threshold_box()):
			ud.prev_state = 'FIND_WALL'
			return 'go_until_obstacle'
		else:
			ud.prev_state = 'FIND_WALL'
			return 'loop'

	def go_to_next_xy(self, ud):
		next_xy = utils.polar_to_cartesian(ud.xyz, 2*pi-((ud.obstacle_direction-pi/2.0)%(2*pi)), 4)
		next_xy = utils.polar_to_cartesian(next_xy, 2*pi-ud.obstacle_direction, 8)
		dth.set_plan_cmd_msg(utils.x_to_lat(next_xy[0]), utils.y_to_lon(next_xy[1]), ud.route_altitude, 1750)
		dth.publish_plan_cmd_msg()

	def check_obstacle(self, ud):
		flag = False
		if(dth.laser_distances[4]>0.3 and dth.laser_distances[4]<8):
			ud.obstacle_direction = dth.orientation_euler[2]
			flag = True
		if(dth.laser_distances[3]>0.3 and dth.laser_distances[3]<8):
			ud.obstacle_direction = (dth.orientation_euler[2] - pi/2.0) % (2*pi)
			flag = True
		if(dth.laser_distances[2]>0.3 and dth.laser_distances[2]<8):
			ud.obstacle_direction = (dth.orientation_euler[2] - pi) % (2*pi)
			flag = True
		if(dth.laser_distances[1]>0.3 and dth.laser_distances[1]<8):
			ud.obstacle_direction = (dth.orientation_euler[2] - 3/2.0*pi) % (2*pi)
			flag = True
		
		return flag
	
	def is_obstacle_relevant(self, ud):
		return not (abs(utils.angle_between_two_directions(ud.obstacle_direction, calcuate_direction_bearing(dth.xyz, ud.planner.get_current_goal(format='xyz')[0])))>pi/2.0)
		
# Change Altitude State
class ChangeAltitude(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['loop', 'plan'], input_keys=['prev_state', 'reason', 'planner', 'route_altitude'], output_keys=['prev_state', 'reason', 'planner'])
	
	def execute(self, ud):
		rospy.loginfo("[CHANGE ALTITUDE] {}".format(ud.reason))
		if(ud.reason == reason.GO_TO_GOAL_ALTITUDE):
			self.go_to_goal_altitude(ud.planner.get_current_goal())
			if(check_threshold_box()):
				ud.prev_state = 'CHANGE_ALTITUDE'
				return 'plan'
			else:
				ud.prev_state = 'CHANGE_ALTITUDE'
				return 'loop'
		elif(ud.reason == reason.ROUTE_ALTITUDE):
			self.go_to_route_altitude(ud.route_altitude)
			if(check_threshold_box()):
				ud.prev_state = 'CHANGE_ALTITUDE'
				return 'plan'
			else:
				ud.prev_state = 'CHANGE_ALTITUDE'
				return 'loop'
		elif(ud.reason == reason.SEARCH):
			self.go_to_search_altitude(ud, ud.planner.get_current_goal())
			if(check_threshold_box()):
				ud.prev_state = 'CHANGE_ALTITUDE'
				return 'plan'
			else:
				ud.prev_state = 'CHANGE_ALTITUDE'
				return 'loop'
		elif(ud.reason == reason.QR_DETECT):
			self.go_to_qr_detect_altitude(ud.planner.get_current_goal())
			if(self.package_threshold()):
				ud.prev_state = 'CHANGE_ALTITUDE'
				return 'plan'
			else:
				ud.prev_state = 'CHANGE_ALTITUDE'
				return 'loop'
		elif(ud.reason == reason.PACKAGE_DELIVERY):
			self.go_to_goal_altitude(ud.planner.get_current_goal())
			if(self.cell_threshold()):
				ud.prev_state = 'CHANGE_ALTITUDE'
				return 'plan'
			else:
				ud.prev_state = 'CHANGE_ALTITUDE'
				return 'loop'

	def package_threshold(self):
		la = False
		lo = False
		alt = False
		
		if abs(dth.plan_cmd_msg.latitude - dth.gps_location[0]) < 0.00000090340708:
			la = True
		if abs(dth.plan_cmd_msg.longitude - dth.gps_location[1]) < 0.00000094973969:
			lo = True
		if abs(dth.plan_cmd_msg.altitude - dth.gps_location[2]) < 0.2:
			alt = True

		if la and lo and alt:
			return True
		
		return False
	
	def cell_threshold(self):
		la = False
		lo = False
		alt = False
		
		if abs(dth.plan_cmd_msg.latitude - dth.gps_location[0]) < 0.00000090340708:
			la = True
		if abs(dth.plan_cmd_msg.longitude - dth.gps_location[1]) < 0.00000094973969:
			lo = True
		if abs(dth.plan_cmd_msg.altitude - dth.gps_location[2]) < 0.2:
			alt = True

		if la and lo and alt:
			return True
		
		return False

	def go_to_goal_altitude(self, goal):
		gps = goal[0]
		# dth.set_plan_cmd_msg(dth.gps_location[0], dth.gps_location[1], gps[2]+0.5, 1750)
		# if(dth.rel_altitude>0.45):
		# 	alt = dth.gps_location[2] + 0.5 - dth.rel_altitude
		# else:
		# 	alt = gps[2] + 0.5
		dth.plan_cmd_msg.altitude = gps[2] + 0.6
		dth.publish_plan_cmd_msg()
	
	def go_to_route_altitude(self, route_altitude):
		dth.set_plan_cmd_msg(dth.gps_location[0], dth.gps_location[1], route_altitude, 1750)
		dth.publish_plan_cmd_msg()
	
	def go_to_search_altitude(self, ud, goal):
		gps = goal[0]

		# if(dth.rel_altitude>10 and dth.rel_altitude <25):
		# 	alt = dth.gps_location[2] + 10 - dth.rel_altitude
		# else:
		# 	alt = gps[2] + 10
		
		if(dth.is_marker_in_scene):
			err_m = dth.err_xy_m
			if((not isnan(err_m[0])) and (not isnan(err_m[1]))):
				goal_xyz = ud.planner.get_current_goal(format='xyz')[0]
			next_xy = [dth.xyz[0] + err_m[0], dth.xyz[1] + err_m[1]]

			dth.set_plan_cmd_msg(utils.x_to_lat(next_xy[0]), utils.y_to_lon(next_xy[1]), gps[2] + 10, 1750)
			dth.publish_plan_cmd_msg(sleep=1)
		else:
			# dth.plan_cmd_msg.altitude = gps[2] + 10
			dth.set_plan_cmd_msg(gps[0], gps[1], gps[2] + 10, 1750)
			dth.publish_plan_cmd_msg()
	
	def go_to_qr_detect_altitude(self, goal):
		gps = goal[0]
		dth.set_plan_cmd_msg(gps[0], gps[1], gps[2] + 2, 1750)
		dth.publish_plan_cmd_msg()

# Sense State
class Sense(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['find_marker'], input_keys=['prev_state'], output_keys=['prev_state', 'ranges'])
	
	def execute(self, ud):
		ud.ranges = self.sense()
		ud.prev_state = 'SENSE'
		return 'find_marker'
	
	def sense(self):
		ranges = [float('Inf'), float('Inf'), float('Inf'), float('Inf')]

		if(dth.laser_distances[4]>0.3):
			ranges[3] = dth.laser_distances[4]
		if(dth.laser_distances[3]>0.3):
			ranges[2] = dth.laser_distances[3]
		if(dth.laser_distances[2]>0.3):
			ranges[1] = dth.laser_distances[2]
		if(dth.laser_distances[1]>0.3):
			ranges[0] = dth.laser_distances[1]

		return ranges

# Find Marker State
class FindMarker(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['loop', 'marker_found', 'search_failed'], input_keys=['prev_state', 'planner', 'ranges', 'search_index'], output_keys=['prev_state', 'planner', 'search_index'])
	
	def execute(self, ud):
		if(self.search_threshold_box()):
			goal_xyz = ud.planner.get_current_goal(format='xyz')[0]
			next_xy = self.diamond_search(ud, goal_xyz)
			if(not len(next_xy)):
				ud.search_index = 0
				ud.prev_state = 'SEARCH'
				return 'search_failed'

			dth.set_plan_cmd_msg(utils.x_to_lat(next_xy[0]), utils.y_to_lon(next_xy[1]), goal_xyz[2]+10, 1750)
			dth.publish_plan_cmd_msg()

		if(dth.is_marker_in_scene):
			ud.prev_state = 'FIND_MARKER'
			return 'marker_found'
		else:
			ud.prev_state = 'FIND_MARKER'
			return 'loop'

	def search_threshold_box(self):
		la = False
		lo = False
		
		if abs(dth.plan_cmd_msg.latitude - dth.gps_location[0]) < 0.0000090340708:
			la = True
		if abs(dth.plan_cmd_msg.longitude - dth.gps_location[1]) < 0.0000094973969:
			lo = True

		if la and lo:
			return True
		
		return False

	def diamond_search(self, ud, goal_xyz):
		ud.search_index += 1
		if(ud.ranges[3]>7 and ud.search_index==1):
			return utils.polar_to_cartesian(goal_xyz, 2*pi-dth.orientation_euler[2], 7)
		elif(ud.ranges[2]>7 and ud.search_index==2):
			return utils.polar_to_cartesian(goal_xyz, 2*pi-((dth.orientation_euler[2]-pi/2.0)%(2*pi)), 7)
		elif(ud.ranges[1]>7 and ud.search_index==3):
			return utils.polar_to_cartesian(goal_xyz, 2*pi-((dth.orientation_euler[2]-pi)%(2*pi)), 7)
		elif(ud.ranges[0]>7 and ud.search_index==4):
			return utils.polar_to_cartesian(goal_xyz, 2*pi-((dth.orientation_euler[2]-3/2.0*pi)%(2*pi)), 7)
		elif(ud.search_index<=4):
			return self.diamond_search(ud, goal_xyz)
		else:
			return []

# Go To Marker State
class GoToMarker(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['loop', 'find_marker', 'search_complete'], input_keys=['prev_state', 'planner', 'search_index'], output_keys=['prev_state', 'planner', 'search_index'])

	def execute(self, ud):
		if(dth.is_marker_in_scene):
			err_m = dth.err_xy_m
			if((not isnan(err_m[0])) and (not isnan(err_m[1]))):
				self.publish_corrected_goal(ud, err_m)

			if(self.marker_threshold_box()):
				ud.search_index = 0
				ud.prev_state = 'SEARCH'
				return 'search_complete'
			else:
				ud.prev_state = 'GO_TO_MARKER'
				return 'loop'
		else:
			ud.prev_state = 'GO_TO_MARKER'
			return 'find_marker'

	def publish_corrected_goal(self, ud, err_m):
		goal_xyz = ud.planner.get_current_goal(format='xyz')[0]
		next_xy = [dth.xyz[0] + err_m[0], dth.xyz[1] + err_m[1]]

		dth.set_plan_cmd_msg(utils.x_to_lat(next_xy[0]), utils.y_to_lon(next_xy[1]), goal_xyz[2]+10, 1750)
		dth.publish_plan_cmd_msg(sleep=1)

	def marker_threshold_box(self):
		la = False
		lo = False

		if abs(dth.plan_cmd_msg.latitude - dth.gps_location[0]) < 1.2*0.00000180681416:
			la = True
		if abs(dth.plan_cmd_msg.longitude - dth.gps_location[1]) < 1.2*0.00000189947939:
			lo = True

		if la and lo:
			return True
		
		return False

# Land State
class Land(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['loop', 'plan'], input_keys=['prev_state', 'planner'], output_keys=['prev_state', 'planner'])
	
	def execute(self, ud):
		gps = ud.planner.get_current_goal()[0]
		dth.set_plan_cmd_msg(gps[0], gps[1], dth.gps_location[2] + (0.25 - dth.rel_altitude), 1750)
		dth.publish_plan_cmd_msg()

		if(check_threshold_box()):
			ud.prev_state = 'LAND'
			return 'plan'
		else:
			ud.prev_state = 'LAND'
			return 'loop'

#Package Acquire State
class PackageAcquire(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['loop', 'package_acquired'], input_keys=['prev_state'], output_keys=['prev_state'])
	
	def execute(self, ud):
		rospy.wait_for_service('/edrone/activate_gripper')
		try:
			if(self.try_acquiring_package()):
				ud.prev_state = 'PACKAGE_RETRIEVAL'
				return 'package_acquired'
			else:
				self.decrease_altitude()
				ud.prev_state = 'PACKAGE_ACQUIRE'
				return 'loop'
		except rospy.ServiceException as e:
			rospy.logerr_throttle(1, "Service call failed: %s" % (e))
			ud.prev_state = 'PACKAGE_ACQUIRE'
			return 'loop'

	def try_acquiring_package(self):
		activate_gripper = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
		res = activate_gripper(True)
		if(res.result):
			return True
		else:
			return False
	
	def decrease_altitude(self):
		dth.plan_cmd_msg.altitude -= 0.05
		dth.publish_plan_cmd_msg()

#Package Deacquire State
class PackageDeacquire(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['loop', 'package_deacquired'], input_keys=['prev_state'], output_keys=['prev_state'])
	
	def execute(self, ud):
		rospy.wait_for_service('/edrone/activate_gripper')
		try:
			if(self.try_deacquiring_package()):
				ud.prev_state = 'PACKAGE_DELIVERY'
				return 'package_deacquired'
			else:
				ud.prev_state = 'PACKAGE_DEAQUIRE'
				return 'loop'
		except rospy.ServiceException as e:
			rospy.logerr_throttle(1, "Service call failed: %s" % (e))
			ud.prev_state = 'PACKAGE_DEAQUIRE'
			return 'loop'
	
	def try_deacquiring_package(self):
		activate_gripper = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
		res = activate_gripper(False)
		if(res.result == False):
			return True
		else:
			return False

#--------------------------------#
# State Machine
#--------------------------------#
def init_state_machine(planner):
	sm = smach.StateMachine(outcomes=[])
	sm.userdata.planner = planner
	sm.userdata.route_altitude = dth.gps_location[2]
	sm.userdata.obstacle_flag = False
	sm.userdata.obstacle_direction = None
	sm.userdata.reason = None
	sm.userdata.search_index = 0

	with sm:
		smach.StateMachine.add('IDLE', Idle(), 
		transitions={'loop':'IDLE', 'plan':'PLAN'})
		
		smach.StateMachine.add('PLAN', Plan(), 
		transitions={'idle':'IDLE', 'takeoff':'TAKEOFF', 'change_altitude':'CHANGE_ALTITUDE', 'en_route':'EN_ROUTE', 'search':'SEARCH', 'land':'LAND', 'package_retrieval':'PACKAGE_RETRIEVAL', 'package_delivery':'PACKAGE_DELIVERY'})
		
		smach.StateMachine.add('TAKEOFF', Takeoff(), 
		transitions={'loop':'TAKEOFF', 'plan':'PLAN'})

		sm_en_route = smach.StateMachine(outcomes=['exit_en_route'], input_keys=['prev_state', 'planner', 'route_altitude', 'obstacle_flag', 'obstacle_direction'], output_keys=['prev_state', 'planner'])

		with sm_en_route:
			smach.StateMachine.add('OBSTACLE_DETECTION', ObstacleDetection(),
			transitions={'go_until_obstacle':'GO_UNTIL_OBSTACLE', 'follow_wall':'FOLLOW_WALL', 'find_wall':'FIND_WALL'})

			smach.StateMachine.add('GO_UNTIL_OBSTACLE', GoUntilObstacle(),
			transitions={'obstacle_detection':'OBSTACLE_DETECTION', 'location_reached':'exit_en_route'})

			smach.StateMachine.add('FOLLOW_WALL', FollowWall(),
			transitions={'obstacle_detection':'OBSTACLE_DETECTION'})

			smach.StateMachine.add('FIND_WALL', FindWall(),
			transitions={'loop':'FIND_WALL', 'go_until_obstacle':'GO_UNTIL_OBSTACLE', 'follow_wall':'FOLLOW_WALL'})
		
		smach.StateMachine.add('EN_ROUTE', sm_en_route,
		transitions={'exit_en_route':'PLAN'})

		smach.StateMachine.add('CHANGE_ALTITUDE', ChangeAltitude(),
		transitions={'loop':'CHANGE_ALTITUDE', 'plan':'PLAN'})

		sm_search = smach.StateMachine(outcomes=['exit_search'], input_keys=['prev_state', 'planner', 'route_altitude', 'search_index'], output_keys=['prev_state', 'planner'])

		with sm_search:
			smach.StateMachine.add('SENSE', Sense(),
			transitions={'find_marker':'FIND_MARKER'})
			smach.StateMachine.add('FIND_MARKER', FindMarker(),
			transitions={'loop':'FIND_MARKER', 'marker_found':'GO_TO_MARKER', 'search_failed':'exit_search'})

			smach.StateMachine.add('GO_TO_MARKER', GoToMarker(),
			transitions={'loop':'GO_TO_MARKER', 'find_marker':'FIND_MARKER', 'search_complete':'exit_search'})
		
		smach.StateMachine.add('SEARCH', sm_search,
		transitions={'exit_search':'PLAN'})

		sm_package_retrieval = smach.StateMachine(outcomes=['package_retrieved'], input_keys=['prev_state'], output_keys=['prev_state'])

		with sm_package_retrieval:
			smach.StateMachine.add('PACKAGE_ACQUIRE', PackageAcquire(),
			transitions={'loop':'PACKAGE_ACQUIRE', 'package_acquired':'package_retrieved'})
		
		smach.StateMachine.add('PACKAGE_RETRIEVAL', sm_package_retrieval,
		transitions={'package_retrieved':'PLAN'})

		sm_package_delivery = smach.StateMachine(outcomes=['package_delivered'], input_keys=['prev_state'], output_keys=['prev_state'])

		with sm_package_delivery:
			smach.StateMachine.add('PACKAGE_DEACQUIRE', PackageDeacquire(),
			transitions={'loop':'PACKAGE_DEACQUIRE', 'package_deacquired':'package_delivered'})
		
		smach.StateMachine.add('PACKAGE_DELIVERY', sm_package_delivery,
		transitions={'package_delivered':'PLAN'})

		smach.StateMachine.add('LAND', Land(),
		transitions={'loop':'LAND', 'plan':'PLAN'})

	return sm

#--------------------------------#
# DRIVER CODE
#--------------------------------#
if __name__ == '__main__':
	rospy.init_node('planner', anonymous=True)
	dth = DroneTopicHandler()
	dth.set_plan_cmd_msg(19.0, 72.0, 8.44, 1750)
	signal.signal(signal.SIGINT, signal_handler)

	warehouse = Warehouse()
	warehouse.set_grid_size(3, 3)
	warehouse.set_reference_location_pickup(18.9998102845, 72.000142461, 16.757981)
	warehouse.set_reference_location_drop(18.9999367615, 72.000142461, 16.757981)
	warehouse.set_depot_location(18.9998887906, 72.0002184402, 16.75)
	warehouse.load('/home/omkar/catkin_ws/src/vitarana_drone/scripts/manifest.csv')

	DPO = DronePathOptimizer()
	DPO.set_cost_matrix(warehouse.to_cost_matrix())
	DPO.run()
	route = DPO.get_optimum_route()
	route.pop(0)
	route.pop()

	dn = 1
	delivery_raw_dict = dict()
	with open('/home/omkar/catkin_ws/src/vitarana_drone/scripts/manifest.csv', 'r') as delivery_file:
		while True:
			line = delivery_file.readline().replace(' ', '').replace('\n', '')
			if(len(line)):
				delivery_raw_dict[dn] = line
				dn += 1
			else:
				break

	print(delivery_raw_dict)
	file_data = ""
	for delivery_number in route:
		file_data += delivery_raw_dict[delivery_number]
		file_data += "\n"
	print(file_data)

	f = open("/home/omkar/catkin_ws/src/vitarana_drone/scripts/sequenced_manifest.csv", "w")
	f.write(file_data)
	f.close()

	planner = FlightPlan()
	planner.set_source((18.9998887906, 72.0002184402, 16.75), 'WAREHOUSE_1')
	for delivery_number in route:
		planner.insert(warehouse.deliveries[delivery_number]['package_retrieval'], goalType.PACKAGE_RETRIEVAL, 'SRC_' + str(delivery_number))
		if(warehouse.deliveries[delivery_number]['package_type'] == 'DELIVERY'):
			planner.insert(warehouse.deliveries[delivery_number]['package_delivery'], goalType.PACKAGE_DELIVERY, 'DEST_' + str(delivery_number))
		elif(warehouse.deliveries[delivery_number]['package_type'] == 'RETURN'):
			planner.insert(warehouse.deliveries[delivery_number]['package_delivery'], goalType.PACKAGE_RETURN, 'DEST_' + str(delivery_number))		
	planner.set_destination((18.9998887906, 72.0002184402, 16.75), 'WAREHOUSE_1')
	planner.activate()

	state_machine = init_state_machine(planner)
	sis = smach_ros.IntrospectionServer('edrone_sm', state_machine, '/planner')
	
	planner.run()
	sis.start()
	state_machine.execute()
	sis.stop()
	
	#--------------------------------------------------------------------------------------------------------------------------------#