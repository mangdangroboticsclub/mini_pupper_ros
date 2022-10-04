from enum import Enum


class CameraType(str, Enum):
    RGB = 'rgb'
    DEPTH = 'depth'
    SIMULATOR = 'simulator'
