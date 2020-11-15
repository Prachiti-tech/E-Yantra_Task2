#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerRequest,TriggerResponse
from vitarana_drone.srv import Gripper
from std_msgs.msg import String

class activate():
    def start(self):
        rospy.init_node('gripper_service') 
        my_service = rospy.Service('/edrone/activate_gripper',Gripper, trigger_response)
        rospy.spin()

    def trigger_response(self,request):
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check)
    
    def gripper_check(self,msg):
        if msg== "True":
            return TriggerResponse(
                success=True,
            )
    def act(self):
        rospy.init_node('sos_service_client')

    # wait for this service to be running
    # Ideally, this service should run 24/7, but remember it's fake :) 
        rospy.wait_for_service('/edrone/activate_gripper')

    # Create the connection to the service. Remeber it's a Trigger service
        gripper_service = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)

    # Create an object of type TriggerRequest. We need a TriggerRequest for a Trigger service
    # We don't need to pass any argument because it doesn't take any
        sos = TriggerRequest()

    # Now send the request through the connection
        result = gripper_service(sos)

    # Done, let's see the result!
        print result
if __name__ == '__main__':

    Activate = activate()
     # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        Activate.start()
        

