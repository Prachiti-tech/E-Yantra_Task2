#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import edrone_cmd
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """
    This is the main class for Vitarana E-Drone.
    It handles
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

        #List of targets setpoints [[Longitude, Latitude, Altitude]]
        self.targets = [    \
                        [72.0, 19.0000000000, 3.00], \
                        [72.0, 19.0000451704, 3.00], \
                        [72.0, 19.0000451704, 0.31]  \
                       ]

        # Kp, Ki and Kd found out experimentally using PID tune
        self.Kp = [0.06*1000*156, 1223* 0.06*1000, 1082*0.06]
        self.Ki = [0.0, 0.0, 0.0*0.008]
        self.Kd = [0.3*10000*873, 2102*0.3*10000, 4476*0.3]

        # Output, Error ,Cummulative Error and Previous Error of the PID equation in the form [Long, Lat, Alt]
        self.error                  = [0.0, 0.0, 0.0]
        self.ouput                  = [0.0, 0.0, 0.0]
        self.cummulative_error      = [0.0, 0.0, 0.0]
        self.previous_error         = [0.0, 0.0, 0.0]
        self.max_cummulative_error  = [1000, 1000, 1000]
        self.throttle               = 0
        self.base_pwm               = 1500
        # ----------------------------------------------------------------------------------------------------------

        # Allowed errors in long.,and lat.+-0.000004517 in latitude,+-0.0000047487
        # We are keeping more precision
        self.allowed_lon_error = 0.0000047487 / 10
        self.allowed_lat_error = 0.0000045170 / 10

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
        self.Ki[0] = long.Ki * 0.008*1000
        self.Kd[0] = long.Kd * 0.3*10000

    # Callback function for latitude tuning in case required
    # This function gets executed each time when /tune_pid publishes /pid_tuning_pitch
    def lat_set_pid(self, lat):
        self.Kp[1] = lat.Kp * 0.06*1000 
        self.Ki[1] = lat.Ki * 0.008*1000
        self.Kd[1] = lat.Kd * 0.3*10000
    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):

        # Error is defined as setpoint minus current orientaion
        self.error[0] = self.location.longitude - self.targets[self.targets_achieved][0]
        self.error[1] = self.location.latitude - self.targets[self.targets_achieved][1]
        self.error[2] = self.location.altitude - self.targets[self.targets_achieved][2]

        for i in range(3):
            # Cummulative error as sum of previous errors
            self.cummulative_error[i] += self.error[i]
            # Limiting the cummulative error
            if abs(self.cummulative_error[i]) >= self.max_cummulative_error[i]:
                self.cummulative_error[i] = 0.0

        # Main PID Equation i.e assigning the output its value acc. to output = kp*error + kd*(error-previous_error) + ki*cummulative_error
        for i in range(3):
            self.ouput[i] = self.Kp[i] * self.error[i] + self.Ki[i] * self.cummulative_error[i] + self.Kd[i] *(self.error[i]-self.previous_error[i])

        # Method for checking the error and then changing the target location
        self.controller()

        # Storing Previous Error values for differential error
        for i in range(3):
            self.previous_error[i] = self.error[i]

        # Setting the throttle that balances the error
        self.drone_cmd.rcThrottle = self.base_pwm - self.ouput[2]

        #Publishing the Drone commands for R,P,Y of drone
        self.drone_pub.publish(self.drone_cmd)

        # Publishing errors for plotjuggler
        self.long_error.publish(self.error[0])
        self.lat_error.publish(self.error[1])
        self.alt_error.publish(self.error[2])
        self.zero_error.publish(0.0)

    def controller(self):

        # Checking if the drone has reached the 1st checkpoint (Altitude of 0.31) and then changing the flag to 1
        # As stated in the problem statement , we have a permissible error of +- 0.05
        if self.location.altitude>self.targets[self.targets_achieved][2]-0.05 and self.location.altitude<self.targets[self.targets_achieved][2]+0.05 and self.targets_achieved == 0 :
            if round(self.previous_error[2],2) == round(self.error[2],2) and round(0.2,2)>abs(self.error[2]):

                # Setting the flag to 2
                self.targets_achieved = 1

                # Specifying the values for R,P,Y
                self.drone_cmd.rcRoll = self.base_pwm - self.ouput[0]
                self.drone_cmd.rcPitch = self.base_pwm - self.ouput[1]
                self.drone_cmd.rcYaw = self.base_pwm

        # Checking if the drone has reached the 2nd checkpoint (long = 72... , lat = 19.000047487) and then changing the flag to 2
        # As stated in the problem statement , we have a permissible error of +- self.allowed_lat_error in latitude and +- self.allowed_lon_error in longitude
        elif self.location.latitude>self.targets[self.targets_achieved][1]-self.allowed_lat_error and self.location.latitude<self.targets[self.targets_achieved][1]+self.allowed_lat_error and self.targets_achieved == 1 :
            if round(self.previous_error[1],8) == round(self.error[1],8) and round(self.allowed_lat_error,10)>abs(self.error[1]):
                if self.location.longitude>self.targets[self.targets_achieved][0]-self.allowed_lon_error and self.location.longitude<self.targets[self.targets_achieved][0]+self.allowed_lon_error and self.targets_achieved == 1:

                    #setting the flag to 2
                    self.targets_achieved = 2

                    #Specifying the values for R,P,Y
                    self.drone_cmd.rcRoll = self.base_pwm
                    self.drone_cmd.rcPitch = self.base_pwm
                    self.drone_cmd.rcYaw = self.base_pwm
        else :
            if self.targets_achieved == 0  :

                #Specifying the values for R,P,Y
                self.drone_cmd.rcRoll = self.base_pwm
                self.drone_cmd.rcPitch = self.base_pwm
                self.drone_cmd.rcYaw = self.base_pwm

            elif self.targets_achieved == 1:

                #Specifying the values for R,P,Y
                self.drone_cmd.rcRoll = self.base_pwm - self.ouput[0]
                self.drone_cmd.rcPitch = self.base_pwm - self.ouput[1]
                self.drone_cmd.rcYaw = self.base_pwm
                # long ---> fwd_bwd ---> roll
                # lat ----> right_left
            elif self.targets_achieved == 2 :
                self.drone_cmd.rcRoll = self.base_pwm - self.ouput[0]
                self.drone_cmd.rcPitch = self.base_pwm - self.ouput[1]
                self.drone_cmd.rcYaw = self.base_pwm


if __name__ == '__main__':
    try:
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
    except:
        pass