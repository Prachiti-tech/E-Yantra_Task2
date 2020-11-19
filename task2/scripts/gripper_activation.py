#!/usr/bin/env python
from __future__ import print_function
import rospy
import sys 
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
from std_msgs.msg import String
from vitarana_drone.srv import *
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #if data.data == "true":
        #rospy.init_node('node_service_server_gripper')
    rospy.wait_for_service('/edrone/activate_gripper')
    act_gripper = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
    req = GripperRequest(True)
    resp = act_gripper(req)
    print("attaching")           
def subscriber():
    rospy.init_node("grip", anonymous=True)
    print("done subscribing")
    rospy.Subscriber('/edrone/gripper_check',String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber()
    
   
    

   
        
                
                
                


                

                



