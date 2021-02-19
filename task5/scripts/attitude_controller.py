#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """
    This is the main class for Vitarana E-Drone attitude controller.
    It handles
        1) Initialization of variables
        2) PID tuning of Roll, Pitch, and Yaw
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

    yaw_set_pid :
        Callback to assign Kp,Ki,Kd for Yaw

    pitch_set_pid :
        Callback to assign Kp,Ki,Kd for pitch

    roll_set_pid :
        Callback to assign Kp,Ki,Kd for roll

    """

    def __init__(self):
        # initializing ros node with name attitude_controller
        rospy.init_node('attitude_controller')

        # Current orientation of eDrone in quaternion format. This value must be updated each time in imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # Current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # The setpoint received from the drone_command in the range from 1000 to 2000 servo-control
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [0.0, 0.0, 0.0]

        # The setpoint of orientation in euler angles to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # Tuned Kp,Ki,Kd for [roll,pitch,yaw]
        # self.Kp = [49.8, 49.8, 0.06*500*46]
        # self.Ki = [0.0, 0.0, 0.008*57]
        # self.Kd = [400.5, 400.5, 0.3*500*15]
        self.Kp = [3593*0.06, 3593*0.06, 5000*0.06]
        self.Ki = [0,   0,    61*0.008]
        self.Kd = [2593*0.3,  2593*0.3,   3100*0.3]

        # -----------------------Add other required variables for pid here ----------------------------------------------

        # Output of PID equation
        self.ouput = [0.0, 0.0, 0.0]
        # Error in PID
        self.error = [0.0, 0.0, 0.0]
        # Integral error
        self.cummulative_error = [0.0, 0.0, 0.0]
        # Variable to store Previous Error
        self.previous_error = [0.0, 0.0, 0.0]
        # Limiting the cummulative error
        self.max_cummulative_error = [100, 100, 100]
        # Limiting the PWM in range 0-1023
        self.max_values = [1024, 1024, 1024, 1024]
        self.min_values = [0, 0, 0, 0]
        # Outputs for roll , pitch and yaw calculated through PID equation
        self.out_roll = 0.0
        self.out_yaw = 0.0
        self.out_pitch = 0.0
        self.throttle = 1000.0

        # ----------------------------------------------------------------------------------------------------------

        # The time in seconds at which PID is to be run
        self.pid_break_time = 0.016

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)

        # ------------------------ROS Publishers for publishing errors-----------------------------------------------------

        self.roll_error_pub = rospy.Publisher(
            '/roll_error', Float32, queue_size=1)
        self.pitch_error_pub = rospy.Publisher(
            '/pitch_error', Float32, queue_size=1)
        self.yaw_error_pub = rospy.Publisher(
            '/yaw_error', Float32, queue_size=1)

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/drone_command', edrone_cmd,
                         self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        # rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        # ------------------------------------------------------------------------------------------------------------

    # Callback function for /edrone/imu/data
    def imu_callback(self, msg):
        # Setting the orientations received from IMU in quaternion
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

    # Callback function for /drone_command
    def drone_command_callback(self, msg):
        # Getting the setpoints and throttle
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.throttle = msg.rcThrottle

    # Callback function for /pid_tuning_roll
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3
        self.Kp[1] = roll.Kp * 0.06
        self.Ki[1] = roll.Ki * 0.008
        self.Kd[1] = roll.Kd * 0.3
        # print("yoo")

    # Callback function for /pid_tuning_pitch
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3
        # print("yoo")

    # Callback function for /pid_tuning_yaw
    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 0.06
        self.Ki[2] = yaw.Ki * 0.008
        self.Kd[2] = yaw.Kd * 0.3
        # print("yoo")

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # Converting quaternion to euler angles
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
            [self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30

        # Converting the range of 1000 to 2000 to 0 to 1024 for throttle
        if self.throttle >= 1000.0 and self.throttle <= 2000.0:
            self.throttle = (self.throttle-1000)*1.024

        # Error is defined as setpoint - current orientation
        for i in range(3):
            self.error[i] = self.setpoint_euler[i] - \
                self.drone_orientation_euler[i]
            # Cummulative error and limiting , resetting it
            self.cummulative_error[i] += self.error[i]
            if abs(self.cummulative_error[i]) > self.max_cummulative_error[i]:
                self.cummulative_error[i] = self.error[i]
        print self.cummulative_error, self.error
        # PID equations for R,P,Y : Output = kp*error + kd*(error-previous_error) + ki*cummulative_error
        for i in range(2):
            self.ouput[i] = self.Kp[i] * self.error[i] + self.Kd[i] * (abs(self.error[i]-self.previous_error[i]) < 30)*(self.error[i]-self.previous_error[i]) + self.Ki[i] * self.cummulative_error[i]

        self.ouput[2] = self.Kp[2] * self.error[2] + self.Kd[2] * \
            (self.error[2]-self.previous_error[2]) + \
            self.Ki[2] * self.cummulative_error[2]
        
        # Storing the current error in previous error for derivative
        for i in range(3):
            self.previous_error[i] = self.error[i]

        # Getting thr roll,pitch and yaw values from output list
        self.out_roll = self.ouput[0]
        self.out_pitch = self.ouput[1]
        self.out_yaw = self.ouput[2]

        # Assigning the appropriate propeller speeds to balance the torque along correct axis , hence attitude control
        self.pwm_cmd.prop1 = max(min(+self.out_roll - self.out_pitch - self.out_yaw + self.throttle, self.max_values[0]), self.min_values[0])
        self.pwm_cmd.prop2 = max(min(-self.out_roll - self.out_pitch + self.out_yaw + self.throttle, self.max_values[1]), self.min_values[1])
        self.pwm_cmd.prop3 = max(min(-self.out_roll + self.out_pitch - self.out_yaw + self.throttle, self.max_values[2]), self.min_values[2])
        self.pwm_cmd.prop4 = max(min(+self.out_roll + self.out_pitch + self.out_yaw + self.throttle, self.max_values[3]), self.min_values[3])

        # Publishing the errors for plotjuggler
        self.yaw_error_pub.publish(self.out_yaw)
        self.pitch_error_pub.publish(self.out_pitch)
        self.roll_error_pub.publish(self.out_roll)

        # Publishing the pwm values on /edrone/pwm
        # print(self.pwm_cmd)
        self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':
    # Waiting for gazebo to start and position controller to start
    # time.sleep(1)

    # Creating an instance of the above created class
    e_drone = Edrone()

    # PID sampling rate
    r = rospy.Rate(1/e_drone.pid_break_time)

    while not rospy.is_shutdown():
        # Calling the pid method
        e_drone.pid()
        # Sleeping for specified sample rate
        r.sleep()

