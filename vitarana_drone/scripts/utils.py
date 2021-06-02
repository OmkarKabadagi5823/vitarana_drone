#!/usr/bin/env python

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
	def angle_between_two_directions(theta1, theta2):
		if(sin(theta1-theta2) > 0):
			return min(abs(theta1 - theta2), abs((theta1 +2 *pi) - theta2))
		else:
			return -min(abs(theta1 - theta2), abs(theta1 - (theta2 + 2*pi)))

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
	
	@staticmethod
	def rotate_axis(x, y, angle):
		x_ = x*cos(angle) + y*sin(angle)
		y_ = -x*sin(angle) + y*cos(angle)

		return [x_, y_]