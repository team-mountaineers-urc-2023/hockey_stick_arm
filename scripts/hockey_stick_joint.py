#!/usr/bin/env python3


from time import time
from time import sleep
import numpy as np

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


class joint_controller:
    def __init__(self) -> None:
        rospy.init_node("roboteq_control")
        self.rate = rospy.Rate(30)

        self.left_mot_axis = 1
        self.right_mot_axis = 4
        self.left_vel = 0
        self.right_vel = 0

        joy_topic = rospy.get_param("~joy_cmd_topic", default="/arm/joy")
        self.joy_sub = rospy.Subscriber(joy_topic, Joy, self.joy_callback)
        self.motor_pub = rospy.Publisher("/arm/motor_cmd", Float32MultiArray, queue_size=1)
        rospy.spin()

    def joy_callback(self, msg: Joy):
        self.left_vel = msg.axes[self.left_mot_axis]
        self.right_vel = msg.axes[self.right_mot_axis]
        print(self.left_vel)
        out_msg = Float32MultiArray()
        out_msg.data = [self.left_vel, self.right_vel]
        self.motor_pub.publish(out_msg)


def main():
    controller = joint_controller()


if __name__ == '__main__':
    main()
