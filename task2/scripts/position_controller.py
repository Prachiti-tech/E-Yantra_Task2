#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import edrone_cmd, location_custom
from sensor_msgs.msg import NavSatFix, LaserScan
from vitarana_drone.srv import Gripper, GripperRequest
from std_msgs.msg import Float32,String
from pid_tune.msg import PidTune
import rospy
import time
import tf
import math
import numpy as np
# class activate():
#     def start(self):
#         rospy.init_node('gripper_service')
#         my_service = rospy.Service('/edrone/activate_gripper',Gripper, trigger_response)
#         rospy.spin()

#     def trigger_response(request):
#         rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check)

#         def gripper_check(self,bool):
#             if self.gripper_check == True:
#                 return TriggerResponse(
#                     success=True,
#                 )
#     def act():
#         rospy.init_node('sos_service_client')

#     # wait for this service to be running
#     # Ideally, this service should run 24/7, but remember it's fake :)
#         rospy.wait_for_service('/edrone/activate_gripper')

#     # Create the connection to the service. Remeber it's a Trigger service
#         gripper_service = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)

#     # Create an object of type TriggerRequest. We need a TriggerRequest for a Trigger service
#     # We don't need to pass any argument because it doesn't take any
#         sos = TriggerRequest()

#     # Now send the request through the connection
#         result = gripper_service(sos)

#     # Done, let's see the result!
#         return result


class Edrone():
    """
    This is the main class for Vitarana E-Drone.
    It handles
    ----------
        1) Initialization of variables
        2) PID tuning of Longitude, Latitude, and Altitude
        3) Callbacks for PID constants
        4) Basic Tasks like initalizing a node and Subscribing

    Note : [ Longitude, Latitude, Altitude ] is the convention followed

    Methods :
    ----------
    pid :
        The main PID algorithm (for Position control) runs when this method is called

    imu_callback :
        Its called when IMU messages are received

    gps_callback :
        Receive the setpoint as in coordinates in 3d space [Latitudes, Longitude, Altitude]

    altitude_set_pid :
        Callback to assign Kp,Ki,Kd for Altitude

    long_set_pid :
        Callback to assign Kp,Ki,Kd for Longitude

    lat_set_pid :
        Callback to assign Kp,Ki,Kd for Latitude

    controller :
        Function for checking if the drone has reached the checkpoint
        and setting a new checkpoint

    """
    def __init__(self):

        # Initializing ros node with name position_control
        rospy.init_node('position_controller')

        #Creating an instance of NavSatFix message and edrone_cmd message
        self.location = NavSatFix()
        self.drone_cmd = edrone_cmd()

        #Created a flag for changing the setpoints
        self.targets_achieved = 0
        # Check if obstacle was deteceted
        self.obstacle_count = 0
        # Gripper data
        self.gripper_data = False
        self.stride = 25
        #List of targets setpoints [[Longitude, Latitude, Altitude]]
        # self.targets = [
        #                 [72.0, 19.0000271036, 3.00], \
        #                 [72.0, 19.000000000, 3.00], \
        #                 [72.0, 19.000000000, 1.0]  \
        #                ]
        self.targets = [
                        [71.9998318945, 19.0009248718, 25.1599967919], \
                        [71.9998955286, 19.0007046575, 25.1599967919], \
                        [71.9998955286, 19.0007046575, 22.1599967919]  \
                       ]
        # Longitude , latitude and altitude
        self.scanned_target = [0.0,0.0,0.0]
        self.range_finder_bottom = 0
        self.bottom_count = 0
        self.offset_alt = 3.00
        # [1] right, [2] back, [3] left, [0,4] front w.r.t eyantra logo
        self.range_finder_top_list = [0.0, 0.0, 0.0 ,0.0, 0.0]
        self.x_lat = 0.0
        self.y_long = 0.0
        # Weights for left right
        self.weights_lr = [1.3,-1.3]
        # self.Activate = activate()
        self.n = -1
        self.points =[]
        # Kp, Ki and Kd found out experimentally using PID tune
        # self.Kp = [0.06*1000*156*4, 1223*0.06*1000*2,1082*0.06]
        # self.Ki = [0.008*10,        0.008*10,         0.0*0.008]
        # self.Kd = [0.3*10000*873*4, 2102*0.3*10000*2,  4476*0.3]
        self.Kp = [0.06*1000*156*4, 1323*0.06*1000*2,1082*0.06]
        self.Ki = [0.008*10,        0.008*10,         0.0*0.008]
        self.Kd = [0.3*10000*873*4, 2202*0.3*10000*2,  4476*0.3]

        # Output, Error ,Cummulative Error and Previous Error of the PID equation in the form [Long, Lat, Alt]
        self.error                  = [0.0, 0.0, 0.0]
        self.ouput                  = [0.0, 0.0, 0.0]
        self.cummulative_error      = [0.0, 0.0, 0.0]
        self.previous_error         = [0.0, 0.0, 0.0]
        self.max_cummulative_error  = [1e-3, 1e-3, 1000]
        self.throttle               = 0
        self.base_pwm               = 1500
        # ----------------------------------------------------------------------------------------------------------

        # Allowed errors in long.,and lat.
        self.allowed_lon_error = 0.0000047487/9
        self.allowed_lat_error = 0.000004517/9

        #Checking if we have to scan or Land
        self.scan = False
        self.gripper = False


        # Time in which PID algorithm runs
        self.pid_break_time = 0.060  # in seconds

        # Publishing servo-control messaages and altitude,longitude,latitude and zero error on errors /drone_command, /alt_error, /long_error, /lat_error
        self.drone_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.alt_error = rospy.Publisher('/alt_error',Float32, queue_size=1)
        self.long_error = rospy.Publisher('/long_error',Float32, queue_size=1)
        self.lat_error = rospy.Publisher('/lat_error',Float32, queue_size=1)
        self.zero_error = rospy.Publisher('/zero_error',Float32, queue_size=1)
        # -----------------------------------------------------------------------------------------------------------

        # Subscribers for gps co-ordinates, and pid_tune GUI
        rospy.Subscriber('/edrone/gps',NavSatFix, self.gps_callback)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.long_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.lat_set_pid)
        rospy.Subscriber('/edrone/location_custom',location_custom,self.scanQR)
        rospy.Subscriber('/edrone/range_finder_bottom',LaserScan,self.range_bottom)
        rospy.Subscriber('/edrone/range_finder_top',LaserScan,self.range_top)
        rospy.Subscriber('/edrone/gripper_check',String, self.gripper_callback)
    # Callback for getting gps co-ordinates
    def gps_callback(self, msg):
        self.location.altitude = msg.altitude
        self.location.latitude = msg.latitude
        self.location.longitude = msg.longitude
        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_altitude in case required
    # This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
    def altitude_set_pid(self, alt):
        self.Kp = alt.Kp * 0.06
        self.Ki = alt.Ki * 0.008
        self.Kd = alt.Kd * 0.3

    # Callback function for longitude tuning in case required
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def long_set_pid(self, long):
        self.Kp[0] = long.Kp * 0.06*1000
        self.Ki[0] = long.Ki * 0.008
        self.Kd[0] = long.Kd * 0.3*10000

    # Callback function for latitude tuning in case required
    # This function gets executed each time when /tune_pid publishes /pid_tuning_pitch
    def lat_set_pid(self, lat):
        self.Kp[1] = lat.Kp * 0.06*1000
        self.Ki[1] = lat.Ki * 0.008*1000
        self.Kd[1] = lat.Kd * 0.3*10000
    #
    def scanQR(self, msg):
        self.scanned_target[0] = msg.longitude
        self.scanned_target[1] = msg.latitude
        self.scanned_target[2] = msg.altitude
        self.scan = msg.scan
        # print self.scanned_target, self.location
    def range_bottom(self , msg):
        self.range_finder_bottom = msg.ranges[0]
        # print(self.range_bottom)

    def range_top(self , msg):
        self.range_finder_top_list = msg.ranges

    def gripper_callback(self,data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        # print data.data
        if data.data == "True":
            self.gripper_data = True
        else :
            self.gripper_data = False
        
    def activate_gripper(self,shall_i):
        rospy.wait_for_service('/edrone/activate_gripper')
        act_gripper = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        req = GripperRequest(shall_i)
        resp = act_gripper(req) 

    def lat_to_x(self, input_latitude):
        self.x_lat = 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        self.y_long= -105292.0089353767 * (input_longitude - 72)
        # if you use this for control, you may have to change the relevant pitch   direction because of the sign

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):

        # Error is defined as setpoint minus current orientaion
        self.error[0] = self.location.longitude - self.targets[self.targets_achieved][0]
        self.error[1] = self.location.latitude - self.targets[self.targets_achieved][1]
        self.error[2] = self.location.altitude - self.targets[self.targets_achieved][2]
        # print self.targets
        # print self.targets_achieved
        # print self.n
        # [71.99986371155, 19.00081476465, 25.1599967919], [71.9998955286, 19.0007046575, 25.1599967919]]
        # [[71.9998318945, 19.0009248718, 25.1599967919], [71.99986371155, 19.00081476465, 25.1599967919], [71.9998955286, 19.0007046575, 25.1599967919], [71.9998955286, 19.0007046575, 25.1599967919], [71.9998955286, 19.0007046575, 22.1599967919]]
        # 19.0008147646

        # print self.error
        # if self.error[1]>=0.0001:
        #     self.error[1] = 0.0001
        # elif self.error[1]<=-0.0001:
        #     self.error[1]=-0.0001
        # if self.error[0]>=9e-5:
        #     self.error[0] = 9e-5
        # elif self.error[0]<=-9e-5:
        #     self.error[0]=-9e-5
        # print self.error
        for i in range(3):
            # Cummulative error as sum of previous errors
            self.cummulative_error[i] += self.error[i]
            # Limiting the cummulative error
            if abs(self.cummulative_error[i]) >= self.max_cummulative_error[i]:
                self.cummulative_error[i] = 0.0

        # Main PID Equation i.e assigning the output its value acc. to output = kp*error + kd*(error-previous_error) + ki*cummulative_error
        for i in range(3):
            self.ouput[i] = self.Kp[i] * self.error[i] + self.Ki[i] * self.cummulative_error[i] + self.Kd[i]*(self.error[i]-self.previous_error[i])

        # Method for checking the error and then changing the target location
        # print self.ouput
        self.controller()
        # self.handle_obstacle_x_y()
        # Storing Previous Error values for differential error
        for i in range(3):
            self.previous_error[i] = self.error[i]

        # Setting the throttle that balances the error
        self.drone_cmd.rcThrottle = self.base_pwm - self.ouput[2]
        if self.drone_cmd.rcThrottle>2000:
            self.drone_cmd.rcThrottle=2000
        elif self.drone_cmd.rcThrottle<1000:
            self.drone_cmd.rcThrottle=1000
        #Publishing the Drone commands for R,P,Y of drone
        self.handle_obstacle_x_y()
        self.drone_pub.publish(self.drone_cmd)

        # Publishing errors for plotjuggler
        self.long_error.publish(self.error[0])
        self.lat_error.publish(self.error[1])
        self.alt_error.publish(self.error[2])
        self.zero_error.publish(0.0)

    def handle_yaw(self):
        if self.location.longitude-self.targets[self.targets_achieved][0] != 0:
            self.drone_cmd.rcYaw = self.base_pwm + math.atan((self.location.latitude-self.targets[self.targets_achieved][1])/(self.location.longitude-self.targets[self.targets_achieved][0]))*180/math.pi
        else:
            self.drone_cmd.rcYaw = self.base_pwm + ((self.location.latitude-self.targets[self.targets_achieved][1])>0)*90 -((self.location.latitude-self.targets[self.targets_achieved][1])<0)*90

    def controller(self):
        # print(self.targets_achieved)
        # print(self.targets)
        # Checking if the drone has reached the 1st checkpoint (Altitude of 0.31) and then changing the flag to 1
        # As stated in the problem statement , we have a permissible error of +- 0.05
        if self.location.altitude>self.targets[self.targets_achieved][2]-0.05 and self.location.altitude<self.targets[self.targets_achieved][2]+0.05 and self.targets_achieved == 0 :
            if round(self.previous_error[2],2) == round(self.error[2],2) and round(0.2,2)>abs(self.error[2]):

                # Setting the flag to 1
                self.targets_achieved += 1
                if not self.gripper_data:
                    self.target_list()

                # Specifying the values for R,P,Y
                # self.handle_yaw()
                self.drone_cmd.rcRoll = self.base_pwm - self.ouput[0]
                self.drone_cmd.rcPitch = self.base_pwm - self.ouput[1]

        # Checking if the drone has reached the 2nd checkpoint (long = 72... , lat = 19.000047487) and then changing the flag to 2
        # As stated in the problem statement , we have a permissible error of +- self.allowed_lat_error in latitude and +- self.allowed_lon_error in longitude
        elif self.location.latitude>self.targets[self.targets_achieved][1]-self.allowed_lat_error and self.location.latitude<self.targets[self.targets_achieved][1]+self.allowed_lat_error and self.targets_achieved <= self.n :
            if round(self.previous_error[1],6) == round(self.error[1],6) and round(self.allowed_lat_error,8)>abs(self.error[1]):
                if self.location.longitude>self.targets[self.targets_achieved][0]-self.allowed_lon_error and self.location.longitude<self.targets[self.targets_achieved][0]+self.allowed_lon_error and self.targets_achieved <= self.n:

                    #setting the flag to 2
                    self.targets_achieved += 1
                    #Specifying the values for R,P,Y
                    # self.drone_cmd.rcRoll = self.base_pwm
                    # self.drone_cmd.rcPitch = self.base_pwm
                    # self.drone_cmd.rcYaw = self.base_pwm
                    self.drone_cmd.rcRoll = self.base_pwm - self.ouput[0]
                    self.drone_cmd.rcPitch = self.base_pwm - self.ouput[1]
        else :
            if self.targets_achieved == 0  :
                print "Taking Off."
                #Specifying the values for R,P,Y
                self.takeoff_control()

            elif self.targets_achieved <= self.n:
                # print "targets achieved {}".format(self.targets_achieved)
                # self.handle_yaw()
                self.drone_cmd.rcRoll = self.base_pwm - self.ouput[0]
                # self.drone_cmd.rcRoll = self.base_pwm
                self.drone_cmd.rcPitch = self.base_pwm - self.ouput[1]
                # self.drone_cmd.rcPitch = self.base_pwm
                # long ---> fwd_bwd ---> roll
                # lat ----> right_left
            elif self.targets_achieved == self.n+1 :
                self.drone_cmd.rcRoll = self.base_pwm - self.ouput[0]
                self.drone_cmd.rcPitch = self.base_pwm - self.ouput[1]
                if self.location.latitude>self.targets[self.targets_achieved][1]-self.allowed_lat_error and self.location.latitude<self.targets[self.targets_achieved][1]+self.allowed_lat_error:
                    if round(self.previous_error[1],7) == round(self.error[1],7) and round(self.allowed_lat_error,8)>abs(self.error[1]):
                        if self.location.longitude>self.targets[self.targets_achieved][0]-self.allowed_lon_error and self.location.longitude<self.targets[self.targets_achieved][0]+self.allowed_lon_error:
                            self.landing_control()
                if self.bottom_count>0:
                    # print self.targets
                    print "Final Target reached"
                    None
                # self.handle_yaw()

    def handle_obstacle_x_y(self):
        # Sum is -ve for right and +ve for left
        front_range_finder_avg = (self.range_finder_top_list[0] + self.range_finder_top_list[4])/2
        # print front_range_finder_avg
        # [1] right, [2] back, [3] left, [0,4] front w.r.t eyantra logo
        # if self.range_finder_top_list[1]<15:
        #     self.drone_cmd.rcYaw = self.base_pwm - self.range_finder_top_list[0]
        #     self.drone_cmd.rcRoll = self.base_pwm + self.range_finder_top_list[0]
        if self.range_finder_top_list[3]<15 and self.gripper_data:
            self.obstacle_count += 1
            self.drone_cmd.rcRoll = self.base_pwm + 10
            self.drone_cmd.rcPitch = self.base_pwm
            self.drone_cmd.rcYaw = self.base_pwm
        if front_range_finder_avg < 13 and self.gripper_data:
            self.obstacle_count += 1
            self.drone_cmd.rcPitch = self.base_pwm - 10
            self.drone_cmd.rcRoll = self.base_pwm
            self.drone_cmd.rcYaw = self.base_pwm
        print self.obstacle_count
        if self.obstacle_count>20:
            self.obstacle_count =0
            # print "Boom"
            # print self.range_finder_top_list[3]
            # print front_range_finder_avg
            self.targets_achieved=1
            self.delete_inserted()
            self.targets[0][0] = self.location.longitude
            self.targets[0][1] = self.location.latitude
            self.target_list()



    def landing_control(self):
        if self.scan == True :
            # print("Checking Gripper")
            if self.bottom_count == 0:
                print "Image Scanned. Targets acquired."
                self.targets[self.targets_achieved][2] -= self.range_finder_bottom
                # print self.gripper_data
                if not self.gripper_data:
                    self.drone_cmd.rcRoll = self.base_pwm-self.ouput[0]
                    self.drone_cmd.rcPitch = self.base_pwm-self.ouput[1]
                    self.drone_cmd.rcYaw = self.base_pwm
                    print "Waiting to get aligned"
                else :
                    self.activate_gripper(shall_i=True)
                    if self.gripper_data:
                        self.targets_achieved = 0
                        # TODO: Delete inserted points
                        # print self.targets_achieved
                        self.delete_inserted()
                        self.target_refresh()
                        # print self.targets
                        self.bottom_count = 1
        elif self.bottom_count>0 :
            self.drone_cmd.rcRoll = self.base_pwm
            self.drone_cmd.rcPitch = self.base_pwm
            self.drone_cmd.rcYaw = self.base_pwm
            if self.gripper_data:
                self.activate_gripper(shall_i=False)
        else :
            self.drone_cmd.rcRoll = self.base_pwm-self.ouput[0]
            self.drone_cmd.rcPitch = self.base_pwm-self.ouput[1]
            self.drone_cmd.rcYaw = self.base_pwm
            
        if self.gripper_data:
            self.allowed_lat_error=0.000004517
            self.allowed_lon_error=0.0000047487
            # TODO: Gripper code
            # self.Activate.start()
            # gripper = self.Activate.act()

        

        if self.bottom_count == 0:
            print "Reached Subtarget. Landing to grab parcel."
            None

    def delete_inserted(self):
        del self.targets[1:self.n]

    def takeoff_control(self):
        #Specifying the values for R,P,Y
        self.drone_cmd.rcRoll = self.base_pwm
        self.drone_cmd.rcPitch = self.base_pwm
        self.drone_cmd.rcYaw = self.base_pwm

    def target_refresh(self):
        # Corodinates of 0th new are co-ordinates of last old +- altitude
        self.targets[0][0] = self.targets[2][0]
        self.targets[0][1] = self.targets[2][1]
        self.targets[1][0] = self.scanned_target[1]
        self.targets[1][1] = self.scanned_target[0]
        self.targets[2][0] = self.scanned_target[1]
        self.targets[2][1] = self.scanned_target[0]
        if self.scanned_target[2] > self.targets[0][2]:
            self.targets[0][2] = self.scanned_target[2] + self.offset_alt
            self.targets[1][2] = self.scanned_target[2] + self.offset_alt
            self.targets[2][2] = self.scanned_target[2]
        else:
            self.targets[0][2] = self.targets[0][2] + self.offset_alt
            self.targets[1][2] = self.targets[0][2]
            self.targets[2][2] = self.scanned_target[2]
        self.target_list()
        # print(self.targets)

    def target_list(self):
        PosX,PosY,alt1 = self.targets[0]
        ToX,ToY,alt1 = self.targets[1]
        dist = math.sqrt(pow((110692.0702932625 *(PosX-ToX)),2)+pow((-105292.0089353767 *(PosY-ToY)),2))
        self.n = int(abs(dist))*self.stride
        # self.n = 2
        points = [[0.0,0.0,0.0] for i in range(self.n)]
        x = float(self.n)
        for i in range(self.n):
            points[i][0] = ((PosX*(1-((i+1)/x))) + ToX*((i+1)/x))
            points[i][1] = ((PosY*(1-((i+1)/x))) + ToY*((i+1)/x))
            points[i][2] = alt1
            self.targets.insert(i+1,points[i][:])
        del self.targets[-2]
            # print(points[i][:])
        # print points
        # print self.targets



if __name__ == '__main__':
    # Waiting for Gazebo to start
    time.sleep(2)

    # Creating an instance of the above class
    e_drone = Edrone()

    # PID sampling rate
    r = rospy.Rate(1/e_drone.pid_break_time)
    while not rospy.is_shutdown():

        # Call pid function
        e_drone.pid()
        # Sleep for specified sampling rate
        r.sleep()
