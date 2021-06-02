#!/usr/bin/env python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vitarana_drone.msg import MarkerData
from sensor_msgs.msg import CameraInfo, LaserScan, Imu, NavSatFix
from utils import utils
import cv2
import numpy as np
import copy
from math import tan, pi, isnan, sqrt
import rospy
import tf

class MarkerDetector():
	def __init__(self):
		rospy.init_node('marker_detector', anonymous=True)
		self.init_props()
		
		rospy.Subscriber('/edrone/camera/image_raw', Image, self.image_callback)
		rospy.Subscriber('/edrone/camera/camera_info', CameraInfo, self.camera_info_callback)
		rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)

		self.marker_data_pub = rospy.Publisher('/edrone/marker_data_raw', MarkerData, queue_size=1)
		
		self.bridge = CvBridge()
		self.cascade_classifier = cv2.CascadeClassifier('/home/omkar/catkin_ws/src/vitarana_drone/scripts/data/cascade.xml')

	#--------------------------------#
	## initializations
	def init_props(self):
		self.img = np.empty([])
		self.marker_data_msg = MarkerData()

		self.err_rp_m = [float('nan'), float('nan')]
		self.err_xy_m = [float('nan'), float('nan')]
		
		self.focal_length = 1e-7 # safeguard from division by zero error
		self.rel_altitude = 0.0
		self.gps_location = [0.0, 0.0, 0.0]
		self.xyz = [0.0, 0.0, 0.0]
		self.orientation_euler = [0.0, 0.0, 0.0]

		self.zero_error = (0, 0) #(0.11, 0.2)

	#--------------------------------#
	## callbacks		
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
			print(e)
			return

	def camera_info_callback(self, msg):
		self.image_size = (msg.height, msg.width)
		self.focal_length = msg.K[0]

	def range_finder_bottom_callback(self, msg):
		if(msg.ranges[0] > 0.4):
			self.rel_altitude = msg.ranges[0]

	def gps_callback(self, msg):
		self.gps_location[0] = msg.latitude
		self.gps_location[1] = msg.longitude
		self.gps_location[2] = msg.altitude

		self.xyz[0] = utils.lat_to_x(msg.latitude)
		self.xyz[1] = utils.lon_to_y(msg.longitude)
		self.xyz[2] = msg.altitude
	
	def imu_callback(self, msg):
		(self.orientation_euler[0], self.orientation_euler[1], self.orientation_euler[2]) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
		self.orientation_euler[2] = utils.imuYaw_to_bearing(self.orientation_euler[2])

	#--------------------------------#
	## utililty
	def set_default_err_xy(self):
		self.marker_data_msg.marker_id = -1
		self.marker_data_msg.err_x_m = float('nan')
		self.marker_data_msg.err_y_m = float('nan')

	def set_err_xy(self, box):
		x, y, w, h = box

		try:
			center_pixel = (x + w/2, y + h/2)
			center_pixel_transformed = (center_pixel[0] - self.image_size[1]/2, -center_pixel[1] + self.image_size[0]/2)

			self.err_rp_m[0] = center_pixel_transformed[0]*self.rel_altitude/self.focal_length# + self.zero_error[0]
			self.err_rp_m[1] = center_pixel_transformed[1]*self.rel_altitude/self.focal_length# + self.zero_error[1]

			drone_x_bearing = (self.orientation_euler[2]+pi/2.0) % (2*pi)
			err_rp_m_copy = copy.deepcopy(self.err_rp_m)
			
			if((not isnan(err_rp_m_copy[0])) and (not isnan(err_rp_m_copy[1]))):
				self.err_xy_m = utils.rotate_axis(err_rp_m_copy[0], err_rp_m_copy[1], -(2*pi-(drone_x_bearing)) % (2*pi))	
				self.marker_data_msg.err_x_m = self.err_xy_m[0]
				self.marker_data_msg.err_y_m = self.err_xy_m[1]

			return self.err_xy_m
		except:
			rospy.logerr("Error in setting err_xy.")
	
	def set_err_xy_from_multiple(self, boxes):
		try:
			minimum = float('inf')

			for x, y, w, h in boxes:
				center_pixel = (x + w/2, y + h/2)
				center_pixel_transformed = (center_pixel[0] - self.image_size[1]/2, -center_pixel[1] + self.image_size[0]/2)

				self.err_rp_m[0] = center_pixel_transformed[0]*self.rel_altitude/self.focal_length# + self.zero_error[0]
				self.err_rp_m[1] = center_pixel_transformed[1]*self.rel_altitude/self.focal_length# + self.zero_error[1]

				drone_x_bearing = (self.orientation_euler[2]+pi/2.0) % (2*pi)
				err_rp_m_copy = copy.deepcopy(self.err_rp_m)
				
				if((not isnan(err_rp_m_copy[0])) and (not isnan(err_rp_m_copy[1]))):
					err_xy_m_local = utils.rotate_axis(err_rp_m_copy[0], err_rp_m_copy[1], -(2*pi-(drone_x_bearing)) % (2*pi))	
					
					dist = sqrt(err_xy_m_local[0]**2 + err_xy_m_local[1]**2)
					val = dist < minimum
					if(val):
						self.err_xy_m[0] = err_xy_m_local[0]
						self.err_xy_m[1] = err_xy_m_local[1]
						minimum = dist
						
			self.marker_data_msg.err_x_m = self.err_xy_m[0]
			self.marker_data_msg.err_y_m = self.err_xy_m[1]
		except:
			rospy.logerr("Error in setting err_xy.")

	#--------------------------------#
	## cascade_classifier
	def harr_cascade(self):		
		self.marker_data_msg.marker_id = -1
		self.marker_data_msg.err_x_m = float('nan')
		self.marker_data_msg.err_y_m = float('nan')
			
		try:
			gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
			marker = self.cascade_classifier.detectMultiScale(gray, scaleFactor=1.05)
			for (x, y, w, h) in marker:
				center_pixel = (x + w/2, y + h/2)
				center_pixel_transformed = (center_pixel[0] - self.image_size[1]/2, -center_pixel[1] + self.image_size[0]/2)
	
				self.err_rp_m[0] = center_pixel_transformed[0]*self.rel_altitude/self.focal_length# + self.zero_error[0]
				self.err_rp_m[1] = center_pixel_transformed[1]*self.rel_altitude/self.focal_length# + self.zero_error[1]

				drone_x_bearing = (self.orientation_euler[2]+pi/2.0) % (2*pi)
				err_rp_m_copy = copy.deepcopy(self.err_rp_m)
				
				if((not isnan(err_rp_m_copy[0])) and (not isnan(err_rp_m_copy[1]))):
					self.err_xy_m = utils.rotate_axis(err_rp_m_copy[0], err_rp_m_copy[1], -(2*pi-(drone_x_bearing)) % (2*pi))	
					self.marker_data_msg.err_x_m = self.err_xy_m[0]
					self.marker_data_msg.err_y_m = self.err_xy_m[1]
		except:
			rospy.logerr("Error in detection.")
		
		self.marker_data_pub.publish(self.marker_data_msg)

	#--------------------------------#
	## custom algorithm	
	def mask_marker(self, img):
		try:
			hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
			white_lower = np.array([0, 0, 220])
			white_upper = np.array([0, 0, 255])
			mask = cv2.inRange(hsv, white_lower, white_upper)
			return cv2.bitwise_and(self.img, self.img, mask=mask)
		except:
			rospy.logerr("Error in masking.")
			return self.img

	def denoise_image(self, img):
		try:
			img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			_, img_thresh = cv2.threshold(img_gray, 127, 255, 0)
			return cv2.dilate(img_thresh, np.ones((7,7),np.uint8), iterations=2)
		except:
			rospy.logerr("Error in denoising.")
			return self.img

	def find_boxes(self, img):
		try:
			contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

			detected_boxes = []
			for contour in contours:
				x, y, w, h = cv2.boundingRect(contour)
				aspect_ratio = float(w)/h
				if(aspect_ratio>=0.75 and aspect_ratio<=1.25 and w*h>=900):
					detected_boxes.append((x, y, w, h))

			return detected_boxes
		except:
			rospy.logerr("Error in finding boxes.")
			return []

	def my_custom_algorithm(self):
		self.set_default_err_xy()

		try:
			masked_img = self.mask_marker(self.img)
			denoised_img = self.denoise_image(masked_img)
			detected_boxes = self.find_boxes(denoised_img)
			
			if(len(detected_boxes)==1):
				self.set_err_xy(detected_boxes[0])
			else:
				self.set_err_xy_from_multiple(detected_boxes)
		except:
			rospy.logerr("Error in detection.")
		
		self.marker_data_pub.publish(self.marker_data_msg)
		

if __name__ == '__main__':
	obj = MarkerDetector()
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
            obj.harr_cascade()
		r.sleep()
