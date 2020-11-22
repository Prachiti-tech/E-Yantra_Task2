#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


from sensor_msgs.msg import Image
from vitarana_drone.msg import location_custom
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import rospy

class image_proc():
	"""
    This is the main class for Vitarana E-Drone.
    It handles
    ----------
        1) Initialization of variables
        2) Decoding the QR Image from the camera and publishing the decoded Data
        3) Callbacks for image conversion
        4) Basic Tasks like initalizing a node, Publishing and Subscribing


    Methods :
    ----------
	ScanCode:
		Function for Scanning the image, decoding the found QR code and publishing the Decoded coordinates

	Image Callback:
		 Callback function for converting the recieved Raw image from the camera to bgr8 image

    """

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
		#Checking if image is a non null value
		if len(self.img.shape) > 1:
			#Decoding the image for finding the QR
			self.decodedObjects = pyzbar.decode(self.img)
			for obj in self.decodedObjects:
				stringSlice = str.split(obj.data, ",")
				try:
					self.list_custom.longitude = float(stringSlice[0])
					self.list_custom.latitude = float(stringSlice[1])
					self.list_custom.altitude = float(stringSlice[2])
					self.list_custom.scan = True
					#Publishing the New found coordinates fr the QR code
					self.location_pub.publish(self.list_custom)
				except :
					pass
				print("Data", stringSlice[0])
				print("Data", stringSlice[1])
				print("Data", stringSlice[2])




	# Callback function of camera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except:
			print("Abcd")
			return

if __name__ == '__main__':
	image_proc_obj = image_proc()
	rate  = rospy.Rate(1/0.06)
	while not rospy.is_shutdown():
		image_proc_obj.ScanCode()
		rate.sleep()
