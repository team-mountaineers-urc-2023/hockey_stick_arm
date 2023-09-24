#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
from math import pi


## This node displays the arm's position on a matplotlib graph for debugging and control assistance

# Create Class to hold the ros node
class Arm_Viz_Node:
    def __init__(self):
        rospy.init_node('arm_viz_node')
        self.rate = rospy.Rate(10)

        # Subscribe to the arm joint_states
        self.joint_state_sub =  rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        # Create a matplotlib figure
        self.fig = plt.figure()

        # Need to save motor positions
        self.shoulder_angle = pi/2
        self.elbow_angle = -pi/2

        # Need the geometry of the arm
        self.l1_length = 1
        self.l2_length = 1

    # Need function for loop
    def loop(self):
        while not rospy.is_shutdown():
            self.draw_arm()
            self.rate.sleep()

    def draw_arm(self):
        # first need to calculate the end points of the arm
        shoulder_location = np.array([0, 0])
        elbow_location = shoulder_location + [self.l1_length * np.cos(self.shoulder_angle), self.l1_length * np.sin(self.shoulder_angle)]
        
        # the angle of the elbow depends on the angle of the shoulder, because the x axis is attached to shoulder
        temp_angle = self.elbow_angle + self.shoulder_angle
        ee_location = elbow_location + [self.l2_length * np.cos(temp_angle), self.l2_length * np.sin(temp_angle)]

        # first clear the figure
        self.fig.clf()
        # Draw the arm
        self.fig.add_subplot(111)
        # Draw the should to elbow joint
        plt.plot([shoulder_location[0], elbow_location[0]], [shoulder_location[1], elbow_location[1]], 'b')
        # Draw the elbow to end joint
        plt.plot([elbow_location[0], ee_location[0]], [elbow_location[1], ee_location[1]], 'r')

        # set axis limits
        plt.xlim((self.l1_length + self.l2_length) * -1.1, (self.l1_length + self.l2_length) * 1.1)
        plt.ylim((self.l1_length + self.l2_length) * -1.1, (self.l1_length + self.l2_length) * 1.1)

        plt.show(block=False)
        plt.pause(0.001)

    # Callback for joint_states that updates the arm position
    def joint_state_callback(self, msg):
        # Get the arm position, this is the shoulder angle and elbow angle, not the motor angles.
        # X axis for shoulder is forward on the robt
        # X axis for elbow is straight out of the shoulder link
        # Elbow should always be less than 90 deg
        self.shoulder_angle = msg.position[0]
        self.elbow_angle = msg.position[1]

def main():
    arm_viz_node = Arm_Viz_Node()
    arm_viz_node.loop()

if __name__ == '__main__':
    main()