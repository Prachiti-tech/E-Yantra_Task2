#!/usr/bin/env python
import cv2
from matplotlib import pyplot as plt
import math
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image,LaserScan
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

class Marker():
    """
    Marker Detection Object : Publishes X,Y error in meters
    """

    def __init__(self):

        rospy.init_node('marker_detection')

        self.hfov_rad = 1.3962634
        self.err_x_m = 10000000.0
        self.err_y_m = 10000000.0
        self.bridge = CvBridge()
        self.logo_cascade = cv2.CascadeClassifier('/home/shreyas/catkin_ws/src/vitarana_drone/scripts/cascade.xml')
        self.img = np.empty([])
        self.logo = None
        self.center_of_box = [0.0,0.0]
        self.Z_m = -1.0
        # Subscribing to the camera topic
        rospy.Subscriber("/edrone/camera/image_raw",Image, self.image_callback)
        rospy.Subscriber('/edrone/range_finder_bottom',LaserScan,self.range_bottom)
        self.x_err_pub = rospy.Publisher("/edrone/err_x_m",data_class=Float32,queue_size=1)
        self.y_err_pub = rospy.Publisher("/edrone/err_y_m",data_class=Float32,queue_size=1)
    
    def range_bottom(self, msg):
        # print msg.ranges[0]
        if not math.isinf(msg.ranges[0]):
            self.Z_m = msg.ranges[0]

    # Callback function of camera topic
    def image_callback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
            self.detect()
        except :
        	print("Error")
        	pass

    def detect(self):
        self.logo = self.logo_cascade.detectMultiScale(self.img, scaleFactor=1.05)
        for (x, y, w, h) in self.logo:
            # print (x,y,w,h)
            self.center_of_box = (w/2+x), (h/2+y)
            self.err_x_m = self.meter_from_pix(self.center_of_box[0],0)
            self.err_y_m = self.meter_from_pix(self.center_of_box[1],1)
        

    def focal_length(self,i):
        return (self.img.shape[i]/2)/math.tan(self.hfov_rad/2)

    def meter_from_pix(self,pix,i):
        try :
            meters = (self.img.shape[i]/2-pix)*self.Z_m/self.focal_length(i)
            return meters
        except :
            pass
            return 0.0

    def pub(self):
        self.x_err_pub.publish(self.err_x_m)
        self.y_err_pub.publish(self.err_y_m)

if __name__ == "__main__" :
    time.sleep(0.5)
    marker = Marker()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        marker.pub()
        rate.sleep()