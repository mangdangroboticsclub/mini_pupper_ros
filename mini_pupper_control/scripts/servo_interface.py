#!/usr/bin/python3

import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory
from MangDang.mini_pupper.HardwareInterface import HardwareInterface

hardware_interface = HardwareInterface()


def callback(msg):
    joint_positions = msg.points[0].positions
    lf1_position = joint_positions[0]
    lf2_position = joint_positions[1]
    lf3_position = joint_positions[2]
    rf1_position = joint_positions[3]
    rf2_position = joint_positions[4]
    rf3_position = joint_positions[5]
    lb1_position = joint_positions[6]
    lb2_position = joint_positions[7]
    lb3_position = joint_positions[8]
    rb1_position = joint_positions[9]
    rb2_position = joint_positions[10]
    rb3_position = joint_positions[11]

    joint_angles = np.array([
        [rf1_position, lf1_position, rb1_position, lb1_position],
        [rf2_position, lf2_position, rb2_position, lb2_position],
        [rf2_position+rf3_position, lf2_position+lf3_position,
         rb2_position+rb3_position, lb2_position+lb3_position]
    ])
    hardware_interface.set_actuator_postions(joint_angles)


def listener():
    rospy.init_node('servo_interface', anonymous=True)
    rospy.Subscriber("/joint_group_position_controller/command",
                     JointTrajectory, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    listener()
