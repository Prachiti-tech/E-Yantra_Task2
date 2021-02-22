#!/usr/bin/env python

# Importing required libraries/modules
import cv2
from matplotlib import pyplot as plt
import math
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image,LaserScan
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import os
import rospkg

class Marker():
    """
    Marker Detection Object : Publishes X,Y error in meters
    """

    def __init__(self):

        rospy.init_node('marker_detection')

        # Given constant for edrone camera
        self.hfov_rad = 1.3962634

        # Initiate x & y errors to large values
        self.err_x_m = 0.0
        self.err_y_m = 0.0

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Importing Cascade xml
        path = rospkg.RosPack().get_path("vitarana_drone")
        path = os.path.join(path,"scripts/cascade.xml")
        self.logo_cascade = cv2.CascadeClassifier(path)

        # Create an empty ndarray
        self.img = np.empty([])

        # Create a variable for storing available detectoins
        self.logo = None

        # List to store center of detected box
        self.center_of_box = [0.0,0.0]

        # Vertical height required for localization using focal length formula
        self.Z_m = -1.0

        # Subscribing to the required topics
        rospy.Subscriber("/edrone/camera/image_raw",Image, self.image_callback)
        rospy.Subscriber('/edrone/range_finder_bottom',LaserScan,self.range_bottom)
        rospy.Subscriber("/alt_diff",Float32,self.alt_diff)

        # Publishers for x and  errors in meters
        self.x_err_pub = rospy.Publisher("/edrone/err_x_m",data_class=Float32,queue_size=1)
        self.y_err_pub = rospy.Publisher("/edrone/err_y_m",data_class=Float32,queue_size=1)
    
    def alt_diff(self,msg):
        self.Z_m = msg.data
    
    # Callback for bottom rangefinder
    def range_bottom(self, msg):
        # Check if range is a feasible number
        if not math.isinf(msg.ranges[0]):
            # self.Z_m = msg.ranges[0]
            None

    # Callback function of camera topic
    def image_callback(self, data):
        try:
            # Converting
            self.img = self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
            self.detect()
        except :
        	print("Error")
        	pass

    def detect(self):

        # Main detection funciton using openCv cascade
        self.logo = self.logo_cascade.detectMultiScale(self.img, scaleFactor=1.05)
        for (x, y, w, h) in self.logo:
            self.center_of_box = (w/2+x), (h/2+y)
            self.err_x_m = self.meter_from_pix(self.center_of_box[0],0)
            self.err_y_m = self.meter_from_pix(self.center_of_box[1],1)
        

    def focal_length(self,i):

        # As per the formula given
        return (self.img.shape[i]/2)/math.tan(self.hfov_rad/2)

    def meter_from_pix(self,pix,i):
        try :
            # As per the formula given
            meters = (self.img.shape[i]/2-pix)*self.Z_m/self.focal_length(i)
            return meters
        except :
            pass
            return 0.0

    def pub(self):
        # Publishing obtained values
        self.x_err_pub.publish(self.err_x_m)
        self.y_err_pub.publish(self.err_y_m)

if __name__ == "__main__" :
    # time.sleep(0.5)
    marker = Marker()
    rate = rospy.Rate(5)
    try : 
        while not rospy.is_shutdown():
            marker.pub()
            rate.sleep()
    except rospy.ROSException as e:
        rospy.logdebug("Exit detection: {}".format(e.message))