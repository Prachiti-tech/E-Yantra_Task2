#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /pid_tuning_altitude
        /pitch_error            /pid_tuning_pitch
        /yaw_error              /pid_tuning_roll
        /edrone/pwm             /edrone/imu/data
                                /edrone/drone_command
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

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

    drone_command_callback :
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
        rospy.init_node('position_controller')  # initializing ros node with name drone_control
        #Creating an instance of NavSatFix message and edrone_cmd message
        self.location = NavSatFix()
        self.drone_cmd = edrone_cmd()
        #Created a flag for changing the setpoints
        self.targets_achieved = 0
        #List of targets setpoints [Longitude, Latitude, Altitude]
        self.targets = [[72.0, 19.000, 3.0],
                        [72.0, 19.0000451704, 3.0],
                        [72.0, 19.0000451704, 0.3]]

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
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.drone_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.alt_error = rospy.Publisher('/alt_error',Float32, queue_size=1)
        self.long_error = rospy.Publisher('/long_error',Float32, queue_size=1)
        self.lat_error = rospy.Publisher('/lat_error',Float32, queue_size=1)
        self.zero_error = rospy.Publisher('/zero_error',Float32, queue_size=1)
        # -----------------------------------------------------------------------------------------------------------

        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.long_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.lat_set_pid)

    # Callback for accepting setpoint coordinate messsages
    def drone_command_callback(self, msg):
        self.location.altitude = msg.altitude
        self.location.latitude = msg.latitude
        self.location.longitude = msg.longitude
        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_altitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
    def altitude_set_pid(self, alt):
        self.Kp = alt.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki = alt.Ki * 0.008
        self.Kd = alt.Kd * 0.3

    # Callback function for longitude tuning
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def long_set_pid(self, long):
        self.Kp[0] = long.Kp * 0.06*1000  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = long.Ki * 0.008*1000
        self.Kd[0] = long.Kd * 0.3*10000

    # Callback function for latitude tuning
    # This function gets executed each time when /tune_pid publishes /pid_tuning_pitch
    def lat_set_pid(self, lat):
        self.Kp[1] = lat.Kp * 0.06*1000  # This is just for an example. You can change the ratio/fraction value accordingly
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
            if abs(self.cummulative_error[i]) >= self.max_cummulative_error[i]:

                self.cummulative_error[i] = 0.0

        # Main PID Equation i.e assigning the output its value acc. to above stated equation
        for i in range(3):
            self.ouput[i] = self.Kp[i] * self.error[i] + self.Ki[i] * self.cummulative_error[i] + self.Kd[i] *(self.error[i]-self.previous_error[i])

        #Method for checking the error and then changing the setpoint
        self.controller()

        # Storing Previous Error values for differential error
        for i in range(3):
            self.previous_error[i] = self.error[i]

        if self.error[2]>=-0.001:
            self.drone_cmd.rcThrottle = 1500 - self.ouput[2]
        elif self.error[2]<0.0 :
            self.drone_cmd.rcThrottle = 1500 - self.ouput[2]


        rospy.loginfo(self.drone_cmd)

        #Publishing the Drone commands for R,P,Y of drone
        self.drone_pub.publish(self.drone_cmd)

        # Publishing errors for plotjuggler
        self.long_error.publish(self.error[0])
        self.lat_error.publish(self.error[1])
        self.alt_error.publish(self.error[2])
        self.zero_error.publish(0.0)

    def controller(self):

        #Checking if the drone has reached the 1st checkpoint (Altitude of 0.3) and then changing the flag to 1
        if self.location.altitude>self.targets[self.targets_achieved][2]-0.1 and self.location.altitude<self.targets[self.targets_achieved][2]+0.1 and self.targets_achieved == 0 :
            if round(self.previous_error[2],2) == round(self.error[2],2) and round(0.2,2)>abs(self.error[2]):

                #setting the flag to 2
                self.targets_achieved = 1

                #Specifying the values for R,P,Y
                self.drone_cmd.rcRoll = 1500 - self.ouput[0]
                self.drone_cmd.rcPitch = 1500 - self.ouput[1]
                self.drone_cmd.rcYaw = 1500

        #Checking if the drone has reached the 2nd checkpoint (long = 72... , lat = 19.000047487) and then changing the flag to 2
        elif self.location.latitude>self.targets[self.targets_achieved][1]-0.00000451704 and self.location.latitude<self.targets[self.targets_achieved][1]+0.00000451704 and self.targets_achieved == 1 :
            if round(self.previous_error[1],8) == round(self.error[1],8) and round(0.00000451704,10)>abs(self.error[1]):
                if self.location.longitude>self.targets[self.targets_achieved][0]-0.0000047487 and self.location.longitude<self.targets[self.targets_achieved][0]+0.0000047487 and self.targets_achieved == 1:

                    #setting the flag to 2
                    self.targets_achieved = 2

                    #Specifying the values for R,P,Y
                    self.drone_cmd.rcRoll = 1500
                    self.drone_cmd.rcPitch = 1500
                    self.drone_cmd.rcYaw = 1500
        else :
            if self.targets_achieved == 0 or self.targets_achieved == 2 :

                #Specifying the values for R,P,Y
                self.drone_cmd.rcRoll = 1500
                self.drone_cmd.rcPitch = 1500
                self.drone_cmd.rcYaw = 1500
            elif self.targets_achieved == 1:

                #Specifying the values for R,P,Y
                self.drone_cmd.rcRoll = 1500 - self.ouput[0]
                self.drone_cmd.rcPitch = 1500 - self.ouput[1]
                self.drone_cmd.rcYaw = 1500
                # long ---> fwd_bwd ---> roll
                # lat ----> right_left


if __name__ == '__main__':

    # Creating an instance of the above class
    e_drone = Edrone()
    # Rate in Hz at which PID should be called
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        # Call pid function
        e_drone.pid()
        # Sleep for 60ms
        r.sleep()
