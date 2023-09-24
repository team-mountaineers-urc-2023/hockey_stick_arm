#!/usr/bin/env python3

from threading import Lock
from time import time
from time import sleep
import numpy as np
import os
from math import pi
import random

import rospy
from serial import Serial, SerialException

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from hockey_stick_arm.srv import Calibrate, CalibrateRequest, CalibrateResponse

from myactuator_control.myactuator_lib import MyActuatorMotor

########################## 
LEFT_USE_CAN = True
RIGHT_USE_CAN = True
########################## 

def main():
    arm_motors_node = arm_motors()
    try:
        arm_motors_node.loop()
    except Exception as e:
        # We really want to make sure that we close the serial port
        # under any circumstance, So catch all errors here
        rospy.logerr(e)

def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(value, lower))

class arm_motors:
    def __init__(self):
        rospy.init_node("roboteq_control")
        self.rate = rospy.Rate(10)
    
        # read launch params
        self.name = rospy.get_param("~name")
        self.motor_left_index = rospy.get_param("~motor_left_index")
        self.motor_right_index = rospy.get_param("~motor_right_index")
        ###########################################################
        #  Added new params for IDs and motor type
        self.motor_L_ID = int(rospy.get_param("~motor_L_ID"),16)    # CAN ID 0x145
        self.motor_R_ID = int(rospy.get_param("~motor_R_ID"),16)    # CAN ID 0x146
        self.motor_type = rospy.get_param("~motor_type")
        # Get Acceleration and Deceleration from YAML
        self.accel = rospy.get_param("~acceleration")
        self.decel = rospy.get_param("~deceleration")
        # Get PID Params from YAML
        self.curr_KP = rospy.get_param("~PID_params/curr_KP")
        self.curr_KI = rospy.get_param("~PID_params/curr_KI")
        self.speed_KP = rospy.get_param("~PID_params/speed_KP")
        self.speed_KI = rospy.get_param("~PID_params/speed_KI")
        self.pos_KP = rospy.get_param("~PID_params/pos_KP")
        self.pos_KI = rospy.get_param("~PID_params/pos_KI")
        # Get params for motor port
        self.motor_L_port = rospy.get_param("~motor_L_port")
        self.motor_R_port = rospy.get_param("~motor_R_port")


        # Create Motor Objects for Left and Right
        self.motor_L = MyActuatorMotor(self.motor_type, self.motor_L_ID, LEFT_USE_CAN, self.motor_L_port)
        self.motor_R = MyActuatorMotor(self.motor_type, self.motor_R_ID, RIGHT_USE_CAN, self.motor_R_port)

        # Until calibration we don't know the correct offsets
        self.m1_offset = 0
        self.m2_offset = 0

        # Read in top speed for motors
        self.max_motor_val = rospy.get_param("~max_speed")
        # Just in case.
        self.stop_all()

        # Write KP and KI values to Motors (From PID Params)
        pid_values = self.motor_L.pid_params_to_value(self.curr_KP,self.curr_KI,self.speed_KP,self.speed_KI,self.pos_KP,self.pos_KI)
        self.motor_L.write_pid("ROM",pid_values[0], pid_values[1], pid_values[2], pid_values[3], pid_values[4], pid_values[5])
        self.motor_R.write_pid("ROM",pid_values[0], pid_values[1], pid_values[2], pid_values[3], pid_values[4], pid_values[5])

        # self.motor_L.write_pid("ROM",self.curr_KP,self.curr_KI,self.speed_KP,self.speed_KI,self.pos_KP,self.pos_KI)
        # self.motor_R.write_pid("ROM",self.curr_KP,self.curr_KI,self.speed_KP,self.speed_KI,self.pos_KP,self.pos_KI)

        # Write Accelerations/Decelerations from Yaml file to motors
        #self.motor_L.write_acceleration(0x02,self.accel)    # Write Speed Control Acceleration
        #self.motor_L.write_acceleration(0x03,self.decel)    # Write Speed Control Deceleration
        #self.motor_R.write_acceleration(0x02,self.accel)    # Write Speed Control Acceleration
        #self.motor_R.write_acceleration(0x03,self.decel)    # Write Speed Control Deceleration
        
        #self.motor_L.write_encoder_as_zero(16384)    # Quarter circle offset
        #self.motor_R.write_encoder_as_zero(16384)    # Quarter circle offset
        #self.motor_ALL = MyActuatorMotor(self.motor_type, self.motor_ALL)
        ############################################################
        self.timeout = rospy.get_param("~timeout")
        self.pose_timeout = 0.5
        
        # Set the ccw_correct for both left and right wheels, which should be opposite of each other
        self.ccw_correct_left = 1
        self.ccw_correct_right = -1

        # create publishers and subscribers
        motor_cmd_topic = rospy.get_param("~motor_cmd_topic")
        #myactuator_data_topic = rospy.get_param("~myactuator_data_topic")
        self.motor_cmd_sub = rospy.Subscriber(motor_cmd_topic, Float32MultiArray, self.motor_cmd_callback)
        #self.data_pub = rospy.Publisher(myactuator_data_topic, MyActuatorData, queue_size=1)    
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self.current_pub = rospy.Publisher("/arm_current", Float32MultiArray, queue_size=1)
        # create a service for calibration
        rospy.Service('calibrate_arm', Calibrate, self.calibrate_callback)

        # track last time motor_cmd was received
        self.last_tick = time()
        self.last_pose = time()


    def calibrate_callback(self, request):
        # Requests should come in in radians, these are the angles that the motors are actually at right now (known angles of some kind).
        # We need to calculate the offset that we are reading so the stated values are correct 

        # Store the calibration values
        requested_m1_angle = (request.value1) * 180 / pi
        requested_m2_angle = (request.value2) * 180 / pi

        # Set temporary offset values to none
        cur_m1_angle = None
        cur_m2_angle = None        

        # Try to read the absolute positions of the motors until they are both read    
        while cur_m1_angle is None or cur_m2_angle is None:
            cur_m1_angle = self.motor_L.read_abs_position()
            cur_m2_angle = self.motor_R.read_abs_position()

        # Calculate the true offsets of the motors
        self.m1_offset =  -cur_m1_angle - requested_m1_angle # Negative sign because measured backwards
        self.m2_offset = cur_m2_angle - requested_m2_angle

        # Return the response to the service
        response = CalibrateResponse()
        response.success = True
        response.message = "Calibration values set to {}, {}".format(requested_m1_angle, requested_m2_angle)
        return response


    def motor_val(self, unit_motor_speed: float, ccw_correction:int) -> int:
        clamped_motor_speed = clamp(unit_motor_speed, -1.0, 1.0)
        motor_val = int(clamped_motor_speed * self.max_motor_val * ccw_correction)
        return motor_val


    def motor_cmd_callback(self, msg: Float32MultiArray):
        # update last motor_cmd received time

        if time() - self.last_tick < 0.01 and (LEFT_USE_CAN==False or RIGHT_USE_CAN==False):
            return
        self.last_tick = time()

        # read unit motor speeds from msg
        motor_left_unit_speed = msg.data[self.motor_left_index]
        motor_right_unit_speed = msg.data[self.motor_right_index]
        # translate unit motor speeds to values motors will accept
        motor_left_val = self.motor_val(motor_left_unit_speed, self.ccw_correct_left)        # Left motor command
        motor_right_val = self.motor_val(motor_right_unit_speed, self.ccw_correct_right)    # Right motor command

        m1_current = self.motor_L.speed_control(motor_left_val, read=True)
        m2_current = self.motor_R.speed_control(motor_right_val, read=True)

        if not m1_current:
            m1_current = 0
        if not m2_current:
            m2_current = 0

        current_msg = Float32MultiArray()
        current_msg.data = [m1_current, m2_current]
        self.current_pub.publish(current_msg)

    def stop_all(self):
        print("Stopping motors")
        self.motor_L.speed_control(0, read=True)
        self.motor_R.speed_control(0, read=True)


    def loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

            # Read the position and publish to joint states
            joint_states = JointState()
            joint_states.name = ['shoulder', 'elbow']

            # The positions are read in degrees
            m1_read_position = self.motor_L.read_abs_position()
            m2_read_position = self.motor_R.read_abs_position()
            # print(m1_read_position, " ", m2_read_position)

            # m1 is left motor is measured with zero straight forward on the robot and positive direction is upward (reference is composite linkage)
            # m2 is right motor is measured with zero straight forward on the robot and positive direction is upward (reference is aluminum tang)

            if m1_read_position and m2_read_position:
                self.last_pose = time()
                # m1_angle is negative position because the motor is backwards
                m1_angle = (-m1_read_position - self.m1_offset) * pi / 180
                m2_angle = (m2_read_position - self.m2_offset) * pi / 180

                # map motor angles from -pi to pi
                m1_angle = (m1_angle + pi % (2*pi)) - pi
                m2_angle = (m2_angle + pi % (2*pi)) - pi
                
                # Find the joint angles from the motor angles
                shoulder_angle = m1_angle # shoulder angle is directly represented by m1
                elbow_angle = m2_angle - m1_angle - pi # el = m2 - m1 - 180 is correct
                
                # Make values easier to read
                shoulder_angle = (shoulder_angle + pi % (2*pi)) - pi # map between -pi and pi
                elbow_angle = (elbow_angle + pi % (2*pi)) - pi # map between - pi and pi

                # Debug
                # print("m1: ", m1_angle * 180 / pi, "m2: ", m2_angle * 180 / pi)
                # print("el: ", elbow_angle * 180 / pi, "sh: ", shoulder_angle * 180 / pi)

                # Publish joint angles as a joint_states message
                joint_states.position = [shoulder_angle, elbow_angle]
                self.joint_state_pub.publish(joint_states)

            # tell robot to stay still if connection lost
            timed_out = (time() - self.last_tick) > self.timeout
            if timed_out:
                print("Timed out: Stopping arm motors, Resume in 1 sec")
                self.stop_all()
                sleep(0.05)

            pose_timed_out = (time() - self.last_pose) > self.pose_timeout
            if pose_timed_out:
                print("Arm pose readings old: Careful using closed loop control")


if __name__ == "__main__":
    main()
