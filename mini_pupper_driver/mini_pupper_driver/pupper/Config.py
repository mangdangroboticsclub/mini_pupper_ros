import numpy as np
import pickle
from pupper.ServoCalibration import MICROS_PER_RAD
from pupper.HardwareConfig import PS4_COLOR, PS4_DEACTIVATED_COLOR
from enum import Enum

# TODO: put these somewhere else
class PWMParams:
    def __init__(self):
        self.pins = np.array([[15, 12, 9, 6], [14, 11, 8, 5], [13, 10, 7, 4]])
        self.range = 4096  ## ADC 12 bits
        self.freq = 250  ## PWM freq


class ServoParams:
    def __init__(self):
        self.neutral_position_pwm = 1500  # Middle position
        self.micros_per_rad = MICROS_PER_RAD  # Must be calibrated
        
        nv_file = "/sys/bus/i2c/devices/3-0050/eeprom"

        # The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
        try:
            with open(nv_file, "rb") as nv_f:
                data = pickle.load(nv_f)  # deserialize a Python object hierarchy from a string of bytes
                matrix = np.array(data['NEUTRAL_ANGLE_DEGREES'])
                print("Get nv calibration params: \n" , matrix)
        except:
            print("Error, get nv calibration params failed, use default value. Please calibrate your pupper !")
            matrix = np.array(
            [[0, 0, 0, 0], [45, 45, 45, 45], [-45, -45, -45, -45]]
            )
        self.neutral_angle_degrees = matrix
        self.servo_multipliers = np.array(
            [[1, 1, -1, -1], [-1, 1, -1, 1], [-1, 1, -1, 1]]
        )

    @property
    def neutral_angles(self):
        return self.neutral_angle_degrees * np.pi / 180.0  # Convert to radians


class Configuration:
    def __init__(self):
        ################# CONTROLLER BASE COLOR ##############
        self.ps4_color = PS4_COLOR
        self.ps4_deactivated_color = PS4_DEACTIVATED_COLOR

        #################### COMMANDS ####################
        self.max_x_velocity = 0.20
        self.max_y_velocity = 0.20
        self.max_yaw_rate = 2
        self.max_pitch = 20.0 * np.pi / 180.0

        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02
        self.z_speed = 0.01  # maximum speed [m/s]
        self.pitch_deadband = 0.02
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.15
        self.roll_speed = 0.16  # maximum roll rate [rad/s] 0.16
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 1.2
        self.max_stance_yaw_rate = 1.5

        #################### STANCE ####################
        self.delta_x = 0.059
        self.delta_y = 0.050
        self.x_shift = 0.00
        self.default_z_ref = -0.08

        #################### SWING ######################
        self.z_coeffs = None
        self.z_clearance = 0.03
        self.alpha = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )

        #################### GAIT #######################
        self.dt = 0.015
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        self.overlap_time = (
            0.08  # duration of the phase where all four feet are on the ground
        )
        self.swing_time = (
            0.07  # duration of the phase when only two feet are on the ground
        )

        ######################## GEOMETRY ######################
        self.LEG_FB = 0.059  # front-back distance from center line to leg axis
        self.LEG_LR = 0.0235  # left-right distance from center line to leg plane
        self.LEG_L2 = 0.060
        self.LEG_L1 = 0.050
        self.ABDUCTION_OFFSET = 0.026  # distance from abduction axis to leg
        self.FOOT_RADIUS = 0.00

        self.HIP_L = 0.0394
        self.HIP_W = 0.0744
        self.HIP_T = 0.0214
        self.HIP_OFFSET = 0.0132

        self.L = 0.176
        self.W = 0.060
        self.T = 0.045

        self.LEG_ORIGINS = np.array(
            [
                [self.LEG_FB, self.LEG_FB, -self.LEG_FB, -self.LEG_FB],
                [-self.LEG_LR, self.LEG_LR, -self.LEG_LR, self.LEG_LR],
                [0, 0, 0, 0],
            ]
        )

        self.ABDUCTION_OFFSETS = np.array(
            [
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
            ]
        )

        ################### INERTIAL ####################
        self.FRAME_MASS = 0.200  # kg
        self.MODULE_MASS = 0.020  # kg
        self.LEG_MASS = 0.010  # kg
        self.MASS = self.FRAME_MASS + (self.MODULE_MASS + self.LEG_MASS) * 4

        # Compensation factor of 3 because the inertia measurement was just
        # of the carbon fiber and plastic parts of the frame and did not
        # include the hip servos and electronics
        self.FRAME_INERTIA = tuple(
            map(lambda x: 3.0 * x, (1.844e-4, 1.254e-3, 1.337e-3))
        )
        self.MODULE_INERTIA = (3.698e-5, 7.127e-6, 4.075e-5)

        leg_z = 1e-6
        leg_mass = 0.010
        leg_x = 1 / 12 * self.LEG_L1 ** 2 * leg_mass
        leg_y = leg_x
        self.LEG_INERTIA = (leg_x, leg_y, leg_z)

    @property
    def default_stance(self):
        return np.array(
            [
                [
                    self.delta_x + self.x_shift,
                    self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                ],
                [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
                [0, 0, 0, 0],
            ]
        )

    ################## SWING ###########################
    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z
        # b_z = np.array([0, 0, 0, 0, self.__z_clearance])
        # A_z = np.array(
        #     [
        #         [0, 0, 0, 0, 1],
        #         [1, 1, 1, 1, 1],
        #         [0, 0, 0, 1, 0],
        #         [4, 3, 2, 1, 0],
        #         [0.5 ** 4, 0.5 ** 3, 0.5 ** 2, 0.5 ** 1, 0.5 ** 0],
        #     ]
        # )
        # self.z_coeffs = solve(A_z, b_z)

    ########################### GAIT ####################
    @property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        return np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks

class SimulationConfig:
    def __init__(self):
        self.XML_IN = "pupper.xml"
        self.XML_OUT = "pupper_out.xml"

        self.START_HEIGHT = 0.3
        self.MU = 1.5  # coeff friction
        self.DT = 0.001  # seconds between simulation steps
        self.JOINT_SOLREF = "0.001 1"  # time constant and damping ratio for joints
        self.JOINT_SOLIMP = "0.9 0.95 0.001"  # joint constraint parameters
        self.GEOM_SOLREF = "0.01 1"  # time constant and damping ratio for geom contacts
        self.GEOM_SOLIMP = "0.9 0.95 0.001"  # geometry contact parameters

        # Joint params
        G = 220  # Servo gear ratio
        m_rotor = 0.016  # Servo rotor mass
        r_rotor = 0.005  # Rotor radius
        self.ARMATURE = G ** 2 * m_rotor * r_rotor ** 2  # Inertia of rotational joints
        # print("Servo armature", self.ARMATURE)

        NATURAL_DAMPING = 1.0  # Damping resulting from friction
        ELECTRICAL_DAMPING = 0.049  # Damping resulting from back-EMF

        self.REV_DAMPING = (
            NATURAL_DAMPING + ELECTRICAL_DAMPING
        )  # Damping torque on the revolute joints

        # Servo params
        self.SERVO_REV_KP = 300  # Position gain [Nm/rad]

        # Force limits
        self.MAX_JOINT_TORQUE = 3.0
        self.REVOLUTE_RANGE = 1.57
