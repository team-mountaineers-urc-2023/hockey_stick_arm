#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from math import sin, cos, atan2, sqrt, pi
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

class TwoLinkArmNode:
    def __init__(self):
        rospy.init_node('two_link_arm_node')
        self.link1_length = 0.44
        self.link2_length = 0.44
        self.base_x = 0.0
        self.base_y = 0.0
        self.current_joint_positions = [90*pi/180, -90*pi/180]
        joy_topic = rospy.get_param("~joy_cmd_topic", default="/arm/joy")
        self.motor_pub = rospy.Publisher("/arm/motor_cmd", Float32MultiArray, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.joy_sub = rospy.Subscriber(joy_topic, Joy, self.joy_callback)

        self.up_down_axis = 1
        self.in_out_axis = 4


        rospy.spin()

    def joint_state_callback(self, msg):
        self.current_joint_positions = msg.position

    def joy_callback(self, msg):
        x_dot = msg.axes[self.in_out_axis]
        y_dot = msg.axes[self.up_down_axis]
        q1 = self.current_joint_positions[0]
        q2 = self.current_joint_positions[1]

        joint_speeds = self.inverse_velocity_kinematics(self.current_joint_positions, self.link1_length, self.link2_length, x_dot, y_dot)
        cmd_msg = Float32MultiArray()

        mot_1_vel = joint_speeds[0]
        mot_2_vel = - joint_speeds[1] - joint_speeds[0] 

        cmd_msg.data = [mot_1_vel, mot_2_vel]
        self.motor_pub.publish(cmd_msg)

    def inverse_velocity_kinematics(self, current_joint_positions, l1, l2, vx_ee, vy_ee):
        c1 = cos(current_joint_positions[0])
        s1 = sin(current_joint_positions[0])
        c2 = cos(current_joint_positions[1])
        s2 = sin(current_joint_positions[1])
        Jacobian = np.array([[-s1*l1 + l2*(-s1*c2 - c1*s2), l2*(-c1*s2 - s1*c2)], \
                             [c1*l1 + l2*(-s1*s2 + c1*c2), l2*(c1*c2 - s1*s2)]])
        joint_speeds = np.linalg.solve(Jacobian, [vx_ee, vy_ee])
        return joint_speeds


def main():
    two_arm_node = TwoLinkArmNode()

if __name__ == "__main__":
    main()