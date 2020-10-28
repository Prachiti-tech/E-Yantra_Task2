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

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [0.0, 0.0, 0.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        # Hint: To see the message structure of prop_speed type the following command in the terminal
        # rosmsg show vitarana_drone/prop_speed

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [49.8, 38.64, 0.06*4000]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [340.5, 553.2, 0.3*5000]
        # -----------------------Add other required variables for pid here ----------------------------------------------
        #
        self.ouput                  = [0.0, 0.0, 0.0]
        self.error                  = [0.0, 0.0, 0.0]
        self.cummulative_error      = [0.0, 0.0, 0.0]
        self.previous_error         = [0.0, 0.0, 0.0]
        self.max_cummulative_error  = [100, 100, 100]
        self.max_values             = [1023 , 1023, 1023, 1023]
        self.min_values             = [0, 0, 0, 0]
        self.out_roll               = 0.0
        self.out_yaw                = 0.0
        self.out_pitch              = 0.0
        self.throttle               = 0.0
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------
        self.roll_error_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.yaw_error_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)
        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        # ------------------------------------------------------------------------------------------------------------

    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.throttle        = msg.rcThrottle
        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_roll
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3

    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3

    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = yaw.Ki * 0.008
        self.Kd[2] = yaw.Kd * 0.3

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        #   1. Convert the quaternion format of orientation to euler angles
        #   2. Convert the setpoin that is in the range of 1000 to 2000 into angles with the limit from -10 degree to 10 degree in euler angles
        #   3. Compute error in each axis. eg: error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0], where error[0] corresponds to error in roll...
        #   4. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        #   5. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        #   6. Use this computed output value in the equations to compute the pwm for each propeller. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        #   7. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        #   8. Limit the output value and the final command value between the maximum(0) and minimum(1024)range before publishing. For eg : if self.pwm_cmd.prop1 > self.max_values[1]:
        #                                                                                                                                      self.pwm_cmd.prop1 = self.max_values[1]
        #   8. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        #   9. Add error_sum to use for integral component

        # Converting quaternion to euler angles
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
        rospy.loginfo(self.drone_orientation_euler)
        # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30

        # Complete the equations for pitch and yaw axis
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30
        # Also convert the range of 1000 to 2000 to 0 to 1024 for throttle here itslef
        if self.throttle >= 1000.0 and self.throttle<=2000.0:
            self.throttle = (self.throttle-1000)*1.023
        
        for i in range (3):
            self.error[i] = self.setpoint_euler[i] - self.drone_orientation_euler[i]
            self.cummulative_error[i] += self.error[i]
            if abs(self.cummulative_error[i]) > self.max_cummulative_error[i] :
                self.cummulative_error[i] = 0

        for i in range (3):
            self.ouput[i] = self.Kp[i] * self.error[i] + self.Kd[i] * (self.error[i]-self.previous_error[i]) + self.Ki[i] * self.cummulative_error[i]

        for i in range(3):
            self.previous_error[i] = self.error[i]
        
        self.out_roll,self.out_pitch,self.out_yaw = self.ouput

        self.pwm_cmd.prop1 = max(min(+self.out_roll - self.out_pitch - self.out_yaw + self.throttle, self.max_values[0]), self.min_values[0])
        self.pwm_cmd.prop2 = max(min(-self.out_roll - self.out_pitch + self.out_yaw + self.throttle, self.max_values[1]), self.min_values[1])
        self.pwm_cmd.prop3 = max(min(-self.out_roll + self.out_pitch - self.out_yaw + self.throttle, self.max_values[2]), self.min_values[2])
        self.pwm_cmd.prop4 = max(min(+self.out_roll + self.out_pitch + self.out_yaw + self.throttle, self.max_values[3]), self.min_values[3])
        # self.pwm_cmd.prop1 = self.setpoint_cmd[0]
        # self.pwm_cmd.prop2 = 512
        # self.pwm_cmd.prop3 = 512
        # self.pwm_cmd.prop4 = 512
        rospy.loginfo(self.ouput)
        rospy.loginfo(self.out_roll)
        self.yaw_error_pub.publish(self.out_yaw)
        self.pitch_error_pub.publish(self.out_pitch)
        self.roll_error_pub.publish(self.out_roll)
        self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
