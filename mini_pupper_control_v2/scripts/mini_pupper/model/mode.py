from enum import Enum


class Mode(str, Enum):
    CALIBRATE = 'calibrate'
    SIT = 'sit'
    STAND = 'stand'
