#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from payload_can.payload_lib import Payload
from time import time, sleep
from threading import Lock

class EE_Dif_Node:
    def __init__(self) -> None:

        # self.shoulder_axis = 1
        # self.elbow_axis = 4
        self.rail_left_axis = 2
        self.rail_right_axis = 5
        self.roll_axis = 6
        self.pitch_axis = 7
        self.gripper_close_but = 4
        self.gripper_open_but = 5

        self.screw_tighten_button = 1
        self.screw_loosen_button = 2
        self.sol_button = 3
        self.prev_sol_state = 0

        self.rail_dir = -1
        self.dif_1_dir = 1
        self.dif_2_dir = 1
        self.gripper_dir = 1
        self.screw_dir = 1

        self.rail_mot_num = 7
        self.dif_1_mot_num = 4
        self.dif_2_mot_num = 5
        self.gripper_mot_num = 6

        self.screw_mot_num = 0
        self.sol_mot_num = 1

        self.rail_speed = 0
        self.roll_speed = 0
        self.pitch_speed = 0
        self.gripper_speed = 0
        self.screw_speed = 0

        self.rail_accel = 0.3
        self.roll_accel = 0.5
        self.pitch_accel = 0.5
        self.gripper_accel = 0.3
        self.screw_accel = 0.3
        #Full power: 
        self.rail_max_speed = 65000
        # self.rail_max_speed = 65525
        # self.dif_max_speed = 65525
        self.gripper_max_speed = 65525
        self.screw_max_speed = 65535

        # Half power
        # self.rail_max_speed = 32767
        # self.dif_max_speed = 32767
        # self.gripper_max_speed = 32767

        # # Quarter power
        # self.rail_max_speed = 16383
        self.dif_max_speed = 16383
        # self.gripper_max_speed = 16383

        self.left_trigger_flag = False
        self.right_trigger_flag = False

        self.payload = Payload()

        rospy.init_node("EE_Dif_Node")
        self.rate = rospy.Rate(10)

        joy_topic = rospy.get_param("~joy_cmd_topic", default="/arm/joy")
        self.joy_sub = rospy.Subscriber(joy_topic, Joy, self.joy_callback)

        # track last time motor_cmd was received
        self.last_tick = time()
        self.timeout = 0.1

    def loop(self):
        while not rospy.is_shutdown():
            if self.last_tick - time() > self.timeout:
                self.stop_all()
                rospy.loginfo("EE Dif Timeout, Stopping")

            self.rate.sleep()

    def stop_all(self):
        # Write to all the motors
        self.write_to_motor(self.rail_mot_type, 0, self.rail_mot_num, self.rail_dir, self.rail_max_speed)
        self.write_to_motor(self.dif_2_mot_type, 0, self.dif_2_mot_num, self.dif_2_dir, self.dif_max_speed)
        self.write_to_motor(self.dif_1_mot_type, 0, self.dif_1_mot_num, self.dif_1_dir, self.dif_max_speed)
        self.write_to_motor(self.gripper_mot_type, 0, self.gripper_mot_num, self.gripper_dir, self.gripper_max_speed)

        

    def joy_callback(self, msg: Joy):
        # don't send commands to the motors more often than every 0.05s
        # if time() - self.last_tick < 0.05:
        #     return
		
		# update last motor_cmd received time
        self.last_tick = time()

        # print("help")
        # Find rail speed

        # check if triggers are pressed

        # before triggers are pressed they read 0 instead of 1, so use trigger pressed flags
        # set these flags if trigger is ever not 0 or 1
        # should not need to check if value is 1, but maybe drivers change, this feels safer
        self.left_trigger_flag = not msg.axes[self.rail_left_axis] in [0, 1]
        self.right_trigger_flag = not msg.axes[self.rail_right_axis] in [0, 1]


        rail_left = msg.axes[self.rail_left_axis] <= 0.95 and\
            self.left_trigger_flag 
         
        rail_right = msg.axes[self.rail_right_axis] <= 0.95 and\
            self.right_trigger_flag 
        
        if rail_left and not rail_right: # if only left is pressed
            self.rail_speed = self.linear_ramp(-(msg.axes[self.rail_left_axis]-1)/2, self.rail_speed, self.rail_accel, 1)
        elif not rail_left and rail_right: # if only right is pressed
            self.rail_speed = self.linear_ramp((msg.axes[self.rail_right_axis]-1)/2, self.rail_speed, self.rail_accel, 1)
        else: # if either none or both are pressed, do nothing
            self.rail_speed = self.linear_ramp(0, self.rail_speed, self.rail_accel, 1)

        # find dif speed
        self.roll_speed = self.linear_ramp(msg.axes[self.roll_axis], self.roll_speed, self.roll_accel, 1)
        self.pitch_speed = self.linear_ramp(-msg.axes[self.pitch_axis], self.pitch_speed, self.pitch_accel, 1)

        mot_1_cmd = max(-1, min(1, -self.roll_speed - self.pitch_speed))
        mot_2_cmd = max(-1, min(1, -self.roll_speed + self.pitch_speed))

        # determine what buttons are being pressed
        gripper_open = msg.buttons[self.gripper_open_but] == 1
        gripper_close = msg.buttons[self.gripper_close_but] == 1

        # If open is pressed and close isnt
        if gripper_open and not gripper_close:
            gripper_cmd = -1
        # if close is pressed and open isnt
        elif not gripper_open and gripper_close:
            gripper_cmd = 1
        else: # otherwise, either both are pressed or none are pressed, set to 0
            gripper_cmd = 0
        
        self.gripper_speed = self.linear_ramp(gripper_cmd, self.gripper_speed, self.gripper_accel, 1)

        # determine what buttons are being pressed
        screw_loosen = msg.buttons[self.screw_loosen_button] == 1
        screw_tighten = msg.buttons[self.screw_tighten_button] == 1

        # If open is pressed and close isnt
        if screw_loosen and not screw_tighten:
            screw_cmd = -1
        # if close is pressed and open isnt
        elif not screw_loosen and screw_tighten:
            screw_cmd = 1
        else: # otherwise, either both are pressed or none are pressed, set to 0
            screw_cmd = 0
        
        self.screw_speed = self.linear_ramp(screw_cmd, self.screw_speed, self.screw_accel, 1)

        if msg.buttons[self.sol_button] == 1 and not self.prev_sol_state:
            self.payload.solenoid_write(self.sol_mot_num, True)
            sleep(0.2)
            # print("solenoid ;)")
            self.prev_sol_state = 1
        self.payload.solenoid_write(self.sol_mot_num, False)
        self.prev_sol_state = msg.buttons[self.sol_button]

        # Write to all the motors
        self.write_to_motor(self.rail_speed, self.rail_mot_num, self.rail_dir, self.rail_max_speed)
        self.write_to_motor(mot_2_cmd, self.dif_2_mot_num, self.dif_2_dir, self.dif_max_speed)
        self.write_to_motor(mot_1_cmd, self.dif_1_mot_num, self.dif_1_dir, self.dif_max_speed)
        self.write_to_motor(self.gripper_speed, self.gripper_mot_num, self.gripper_dir, self.gripper_max_speed)
        self.write_to_motor(self.screw_speed, self.screw_mot_num, self.screw_dir, self.screw_max_speed)



    def linear_ramp(self, cmd, cur_speed, accel_limit, max_speed):
        # If the commad says go faster
        if cur_speed < cmd:
            # Set the command to the cur speed + accel limit unless that overshoots
            cmd_speed = min(cmd, cur_speed + accel_limit)
        
        # If the command says go slower
        elif cur_speed > cmd:
            # Set the command to the cur speed - accel limit unless that overshoots
            cmd_speed = max(cmd, cur_speed - accel_limit)
        else:
            # If we are already there, just stay
            cmd_speed = cmd
        
        # Clamp between max and min value for speed
        cmd_speed = max(-max_speed, min(max_speed, cmd_speed))
        return cmd_speed

    def write_to_motor(self, cmd, mot_num, mot_dir, mot_max_speed):
        # Write to the motor but convert a speed [-1,1] to [-max_speed, max_speed] and handle the motor type problem
        self.payload.DC_write(mot_num, int(mot_dir * cmd * mot_max_speed))
        

def main():
    ee_dif_node = EE_Dif_Node()
    ee_dif_node.loop()

if __name__ == "__main__":
    main()
