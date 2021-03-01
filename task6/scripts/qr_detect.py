#!/usr/bin/env python

from sensor_msgs.msg import Image
from vitarana_drone.msg import location_custom
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import rospy

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode
		self.img = np.empty([]) # This will contain your image frame from camera
		self.decodedObjects = []
		rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.location_pub = rospy.Publisher("/edrone/location_custom",location_custom,queue_size=1)
		self.bridge = CvBridge()
		self.list_custom = location_custom()


	def ScanCode(self):
		if len(self.img.shape) > 1:
			self.decodedObjects = pyzbar.decode(self.img)
			for obj in self.decodedObjects:
				stringSlice = str.split(obj.data, ",")
				self.list_custom.longitude = float(stringSlice[0])
				self.list_custom.latitude = float(stringSlice[1])
				self.list_custom.altitude = float(stringSlice[2])
				self.list_custom.scan = True
				self.location_pub.publish(self.list_custom)
				print("Data", stringSlice[0])
				print("Data", stringSlice[1])
				print("Data", stringSlice[2])

	# Callback function of camera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except:
			print("Error")
			return

if __name__ == '__main__':
	image_proc_obj = image_proc()
	rate  = rospy.Rate(1.0/0.060)
	while not rospy.is_shutdown():
		try:
			image_proc_obj.ScanCode()
			rate.sleep()
		except:
			pass