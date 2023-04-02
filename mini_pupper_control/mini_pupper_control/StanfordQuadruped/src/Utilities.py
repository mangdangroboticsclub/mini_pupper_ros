# This program is based on
# https://github.com/stanfordroboticsclub/StanfordQuadruped/blob/master/src/Utilities.py
# which is released under the MIT License.
# Copyright (c) 2020 Stanford Student Robotics
# https://github.com/stanfordroboticsclub/StanfordQuadruped/blob/62277f64dc4d1b293feddc8ecd0986d144f656d6/LICENSE


import numpy as np


def deadband(value, band_radius):
    return max(value - band_radius, 0) + min(value + band_radius, 0)


def clipped_first_order_filter(input, target, max_rate, tau):
    rate = (target - input) / tau
    return np.clip(rate, -max_rate, max_rate)
