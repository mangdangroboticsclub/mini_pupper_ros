import numpy as np


class Command:
    """Stores movement command
    """

    def __init__(self):
        self.horizontal_velocity = np.array([0, 0])
        self.yaw_rate = 0.0
        self.height = -0.06 #default -0.07
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0

        self.hop_event = False
        self.trot_event = False # default trotting, not resting
        self.activate_event = False # default actived without ps4 joystick
        self.dance_activate_event = False

        self.dance_switch_event = False
        self.gait_switch_event = False
        
        self.shutdown_signal = False