#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from math import sin, cos, atan2, sqrt, pi
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from hockey_stick_arm.srv import Calibrate, CalibrateRequest, SetFloat, SetFloatRequest, SetFloatResponse
from typing import Tuple

class TwoLinkArmNode:
    def __init__(self):
        # Init ROS Node
        rospy.init_node('two_link_arm_node')
        
        # Add link lengths so IJ works properly
        self.link1_length = 0.44
        self.link2_length = 0.44
        
        # Set the x, y location of the base to 0, 0 (center of motor shafts)
        self.base_x = 0.0
        self.base_y = 0.0
        
        # Setup Joystick Constants
        self.up_down_axis = 1 # Up down axis is the joystick axis to control up and down motion
        self.in_out_axis = 4 # In out axis is the joystickk axis that moves the arm forward and backwards
        self.left_mot_axis = 1
        self.right_mot_axis = 4


        # Initialize mode to joint_control
        self.in_velo_control = False

        # Initialize joint_positions to None until we switch to velo control mode the first time
        self.current_joint_positions = None

        # Read what topic to receive joy messages from
        joy_topic = rospy.get_param("~joy_cmd_topic", default="/arm/joy")

        # Create a publisher for commands to the motors
        self.motor_pub = rospy.Publisher("/arm/motor_cmd", Float32MultiArray, queue_size=1)

        # Create a service that switches from joint_control to velo control
        rospy.Service('UseVeloControl', SetBool, self.use_velo_control_callback)

        # make a service to set the speed multiplyer
        rospy.Service('SetSpeedMultiplier', SetFloat, self.set_speed_multiplyer_callback)
        self.speed_multiplyer = 1.0 # Speed multiplyer is how fast the arm moves

        # make a service to set the safety check
        rospy.Service('UseArmSafetyCheck', SetBool, self.use_safety_check_callback)
        self.use_safety = True

        # Create a subscriber to the joint states of the arm and the joystick info
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.joy_sub = rospy.Subscriber(joy_topic, Joy, self.joy_callback)

        # Create a service proxy for the Calibrate service
        rospy.wait_for_service('calibrate_arm')
        self.calibrate_proxy = rospy.ServiceProxy('calibrate_arm', Calibrate)

        # Spin and let callbacks handle the rest
        rospy.spin()

    # Joint state callback saves the position of the arm
    def joint_state_callback(self, msg):
        # If we have switched to velo control and therfore calibrated already we can set current_joint_positions
        if self.current_joint_positions:
            self.current_joint_positions = msg.position

    # Joy callback does all the main processing
    def joy_callback(self, msg):
        # If we are in velocity control
        if self.in_velo_control:
            # Set x_dot (x velocity) to the in out axis and y_dot (y velocity) to the up_down axis
            x_dot = self.speed_multiplyer * msg.axes[self.in_out_axis]
            y_dot = self.speed_multiplyer * msg.axes[self.up_down_axis]

            # Calculate joint speeds using the inverse jacobian function
            joint_speeds = self.inverse_velocity_kinematics(self.current_joint_positions, self.link1_length, self.link2_length, x_dot, y_dot)
            
            # Motor speeds aren't exactly the same as joint speeds becuase of our setup
            mot_1_vel = joint_speeds[0]
            mot_2_vel = - joint_speeds[1] - joint_speeds[0] 

            # check safety
            if self.use_safety:
                m1_good, m2_good = self.safety_check(shoulder_angle=self.current_joint_positions[0], elbow_angle=self.current_joint_positions[1], m1_cmd=mot_1_vel, m2_cmd=mot_2_vel)

            # Create a blank message to send out the command, fill it in and send
            cmd_msg = Float32MultiArray()
            if m1_good and m2_good:
                cmd_msg.data = [mot_1_vel, mot_2_vel]
            else:
                cmd_msg.data = [0, 0]
            self.motor_pub.publish(cmd_msg)
        else: # If we are in joint control mode
            # Read joint velocity values from the joystick input
            self.left_vel = self.speed_multiplyer * msg.axes[self.left_mot_axis]
            self.right_vel = self.speed_multiplyer * msg.axes[self.right_mot_axis]
            
            if self.current_joint_positions: # If we are calibrated
                # check safety
                if self.use_safety:
                    m1_good, m2_good = self.safety_check(shoulder_angle=self.current_joint_positions[0], elbow_angle=self.current_joint_positions[1], m1_cmd=self.left_vel, m2_cmd=self.right_vel)

                cmd_msg = Float32MultiArray()
                cmd_msg.data = [self.left_vel * m1_good, self.right_vel * m2_good]
                self.motor_pub.publish(cmd_msg)
            else: # We are not calibrated and can't know what is safe
                # Create the output message, fill it in and send it
                cmd_msg = Float32MultiArray()
                cmd_msg.data = [self.left_vel, self.right_vel]
                self.motor_pub.publish(cmd_msg)

    def use_velo_control_callback(self, request):
        # Set flag based on the request
        self.in_velo_control = request.data

        # if we have not set our current_joint_position yet
        if not self.current_joint_positions:
            # Set to the inverse L shape
            # The request values represent motor angles, not joint angles
            # L shape is shoulder motor straight up and aluminum tang straight back
            self.current_joint_positions = [90*pi/180, 180*pi/180]

            # Create a request message for the calibration service
            request = CalibrateRequest()
            request.value1 = self.current_joint_positions[0]
            request.value2 = self.current_joint_positions[1]

            # Call the calibrations service using the service proxy
            response = self.calibrate_proxy(request)

        # Return the response
        response = SetBoolResponse()
        response.success = True
        response.message = "Flag set to {}".format(self.in_velo_control)
        return response
    
    def set_speed_multiplyer_callback(self, request: SetFloatRequest):
        if request.value < 0.1:
            request.value = 0.1
        elif request.value > 1:
            request.value = 1
        self.speed_multiplyer = request.value
        response = SetFloatResponse()
        response.success = True
        response.message = "Speed multiplyer set to {}".format(self.speed_multiplyer)
        return response

    # This function creates joint speeds from an end effector velocity command
    def inverse_velocity_kinematics(self, current_joint_positions, l1, l2, vx_ee, vy_ee):
        # Precalculate the consines and sines of the two joint angles
        c1 = cos(current_joint_positions[1])
        s1 = sin(current_joint_positions[1])
        c2 = cos(current_joint_positions[0])
        s2 = sin(current_joint_positions[0])

        # Calculate the jacobain matrix
        Jacobian = np.array([[-s1*l1 + l2*(-s1*c2 - c1*s2), l2*(-c1*s2 - s1*c2)], \
                             [c1*l1 + l2*(-s1*s2 + c1*c2), l2*(c1*c2 - s1*s2)]])
        
        # Solve the linear equation relating joint speed and ee speed through the jacobian
        joint_speeds = np.linalg.solve(Jacobian, [vx_ee, vy_ee])
        
        # send out the joint speeds
        return joint_speeds

    def safety_check(self, shoulder_angle, elbow_angle, m1_cmd, m2_cmd) -> Tuple:
        # We want to return whether or not either portion of the move is allowed    
        # Condition 1) m1 should always be less than 110 degrees
        # Condition 2) m2 - m1 should be > 40 degrees
        # Condition 3) m2 - m1 should be < 160 degrees
        m1_max = 105 * pi/180
        m2_m1_max_dif = 145 * pi/180
        m2_m1_min_dif = 40 * pi/180


        # first need to calculate motor angles from joint angles
        m1, m2 = self.motor_angle_from_joint_angle(shoulder_angle, elbow_angle)


        # next need to predict motor angles 0.1 seconds into future
        m1_next = 0.1 * m1_cmd + m1
        m2_next = 0.1 * m2_cmd + m2

        m2_m1_dif = m2 - m1
        m2_m1_next_dif = m2_next - m1_next


        # There are a bunch of cases for if we allow or dissalow moves
        # if all conditions are good, only allow moves that continue to be good
        # if condition 1 is bad only allow moves that make it better and don't make 2 or 3 bad
        # if condition 2 is bad only allow mvoes that make it better and don't make 1 or 3 worse
        # if condition 3 is bad only allow moves that make it better and don't make 1 or 2 worse
        # In checking, if a move is set to false, never re-allow it

        # Start off by allowing all moves and then disallow them by checking conditions
        m1_increase = True
        m1_decrease = True
        m2_increase = True
        m2_decrease = True

        # First, if we are good now, we only allow motion if all conditions in the next step are good too
        if m1 < m1_max and m2_m1_dif < m2_m1_max_dif or m2_m1_dif > m2_m1_min_dif:
            if m1_next > m1_max:
                m1_increase = False
            if m2_m1_next_dif > m2_m1_max_dif:
                m1_decrease = False
                m2_increase = False
            if m2_m1_next_dif < m2_m1_min_dif:
                m1_increase = False
                m2_decrease = False
        
        # If one condition is bad, we don't allow any motion that makes a condition worse:
        # This may allow the other condition to dip into a disallowed region, but only breifly
        # If condition 1 is bad, don't allow m1 to increase 
        if m1 > m1_max:
            m1_increase = False

        # If condition 2 is bad, don't allow m1 to increase or m2 to decrease
        if m2_m1_dif < m2_m1_min_dif:
            m1_increase = False
            m2_decrease = False

        # If condition 3 is bad, don't allow m1 to decrease or m2 to increase
        if m2_m1_dif > m2_m1_max_dif:
            m1_decrease = False
            m2_increase = False

        m1_allowed, m2_allowed = False, False

        if m1_cmd < 0 and m1_increase:
            m1_allowed = True
        if m1_cmd > 0 and m1_decrease:
            m1_allowed = True
        if m2_cmd < 0 and m2_increase:
            m2_allowed = True
        if m2_cmd > 0 and m2_decrease:
            m2_allowed = True

        # print("-------------------------------------------------------")
        # print("m1: ", m1, " m2: ", m2, " m2-m1: ", m2_m1_dif)
        # print("m1_next: ", m1_next, " m2_next: ", m2_next, " m2n-m1n: ", m2_m1_next_dif)
        # print("m1a: ", m1_allowed, "m2a: ", m2_allowed)
        # print("m1_max: ", m1_max, " m1_m2_dif_max: ", m2_m1_max_dif, " m1_m2_dif_min: ", m2_m1_min_dif)
        # print("-------------------------------------------------------")

        return (m1_allowed, m2_allowed)

    def use_safety_check_callback(self, request: SetBoolRequest):
        self.use_safety_check = request.value
        response = SetBoolResponse()
        response.success = True
        response.message = "Safety check set to {}".format(self.use_safety_check)
        return response

    def motor_angle_from_joint_angle(self, shoulder_angle, elbow_angle):
        m1_angle = shoulder_angle
        m2_angle = elbow_angle + m1_angle + pi
        return (m1_angle, m2_angle)
    


def main():
    two_arm_node = TwoLinkArmNode()

if __name__ == "__main__":
    main()