#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vitarana_drone.srv import goal_gps, goal_gpsRequest, goal_gpsResponse
import cv2
import numpy as np
import rospy
from pyzbar import pyzbar

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode 
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.img = np.empty([]) # This will contain your image frame from camera
		self.decoded_msg = []
		self.bridge = CvBridge()

		self.qr_detect_service = rospy.Service('/edrone/capture_qr', goal_gps, self.service_callback)


	# Callback function of amera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except CvBridgeError as e:
			print(e)
			return
	
	def qr_detection(self):
		try:
			barcodes = pyzbar.decode(self.img)
			if(len(barcodes)):
				self.decoded_msg = str(barcodes[0].data.decode('utf-8')).split(',')
				rospy.loginfo(self.decoded_msg)
				return True
		except:
			rospy.logerr("Unkown error occured.")
			return False
	
	def service_callback(self, req):
		rospy.loginfo("QR capture request: {}".format(req.capture_qr))
		res = goal_gpsResponse()
		if(req.capture_qr):
			if(self.qr_detection()):
				res.latitude = float(self.decoded_msg[0])
				res.longitude = float(self.decoded_msg[1])
				res.altitude = float(self.decoded_msg[2])
				res.success = True
			else:
				res.latitude = float('Inf')
				res.longitude = float('Inf')
				res.altitude = float('Inf')
				res.success = False
				
			rospy.loginfo("Service request success: {}".format(res.success))
			
			if(res.success):
				rospy.loginfo("Goal:\nlatitude: {}\nlongitude: {}\naltitude: {}".format(\
					res.latitude, res.longitude, res.altitude))
					
			return res

if __name__ == '__main__':
	image_proc_obj = image_proc()
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		r.sleep()
	
	rospy.spin()