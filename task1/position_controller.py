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
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control

        self.location = NavSatFix()
        self.drone_cmd = edrone_cmd()
        self.target_altitude = 3.0
        self.Kp = 860*0.06
        self.Ki = 0.0*0.008
        self.Kd = 2184*0.3
        # -----------------------Add other required variables for pid here ----------------------------------------------
        #
        self.error                  = 0.0
        self.ouput                  = 0.0
        self.cummulative_error      = 0.0
        self.previous_error         = 0.0
        self.max_cummulative_error  = 1000
        self.throttle               = 0
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.drone_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.alt_error = rospy.Publisher('/alt_error',Float32, queue_size=1)
        self.zero_error = rospy.Publisher('/zero_error',Float32, queue_size=1)
        # -----------------------------------------------------------------------------------------------------------
        rospy.Subscriber('/edrone/gps',NavSatFix, self.drone_command_callback)
        # rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)

    def drone_command_callback(self, msg):
        self.location.altitude = msg.altitude
        self.location.latitude = msg.latitude
        self.location.longitude = msg.longitude
        # ---------------------------------------------------------------------------------------------------------------

    # def altitude_set_pid(self, alt):
    #     self.Kp = alt.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
    #     self.Ki = alt.Ki * 0.008
    #     self.Kd = alt.Kd * 0.3

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        self.error = (self.location.altitude - self.target_altitude)
        self.cummulative_error += self.error
        if abs(self.cummulative_error) >= self.max_cummulative_error:
            self.cummulative_error = 0.0
        
        self.ouput = self.Kp * self.error + self.Ki * self.cummulative_error + self.Kd *(self.error-self.previous_error)
        self.previous_error = self.error
        
        if self.error>=-0.001:
            self.drone_cmd.rcThrottle = 1500 - self.ouput
        elif self.error<0.0 :
            self.drone_cmd.rcThrottle = 1500 - self.ouput
        self.drone_cmd.rcPitch = 1500
        self.drone_cmd.rcRoll = 1500
        self.drone_cmd.rcYaw = 1500
        rospy.loginfo(self.error)
        rospy.loginfo(self.drone_cmd)
        self.drone_pub.publish(self.drone_cmd)
        self.alt_error.publish(self.error)
        self.zero_error.publish(0.0)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
