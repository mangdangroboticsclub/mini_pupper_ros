# This program is based on
# https://github.com/stanfordroboticsclub/StanfordQuadruped/blob/master/src/State.py
# which is released under the MIT License.
# Copyright (c) 2020 Stanford Student Robotics
# https://github.com/stanfordroboticsclub/StanfordQuadruped/blob/62277f64dc4d1b293feddc8ecd0986d144f656d6/LICENSE


import numpy as np
from enum import Enum


class State:
    def __init__(self):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        self.height = -0.07
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0
        self.behavior_state = BehaviorState.REST

        self.ticks = 0
        self.foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))

        self.behavior_state = BehaviorState.REST


class BehaviorState(Enum):
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    HOP = 2
    FINISHHOP = 3
    SHUTDOWN = 96
