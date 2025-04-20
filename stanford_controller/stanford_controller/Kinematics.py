import numpy as np


def leg_explicit_inverse_kinematics(r_body_foot, leg_index, config):
    """
    Find the joint angles for one leg given its body-relative foot position.

    Parameters
    ----------
    r_body_foot : array_like
        Foot position (x, y, z) in the body frame.
    leg_index : int
        Index of the leg (0-based).
    config : Config
        Robot configuration parameters.

    Returns
    -------
    numpy.ndarray
        Joint angles for the specified leg (3,).

    """
    (x, y, z) = r_body_foot

    # Distance from the leg origin to the foot, projected into the y-z plane
    R_body_foot_yz = (y ** 2 + z ** 2) ** 0.5

    # Distance from the leg's forward/back point of rotation to the foot
    R_hip_foot_yz = (R_body_foot_yz ** 2 - config.ABDUCTION_OFFSET ** 2) ** 0.5

    # Interior angle of the right triangle formed in the y-z plane by the leg
    # that is coincident to the ab/adduction axis
    # For feet 2 (front left) and 4 (back left), the abduction offset is
    # positive, for the right feet, the abduction offset is negative.
    arccos_argument = config.ABDUCTION_OFFSETS[leg_index] / R_body_foot_yz
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    phi = np.arccos(arccos_argument)

    # Angle of the y-z projection of the hip-to-foot vector, relative to the
    # positive y-axis
    hip_foot_angle = np.arctan2(z, y)

    # Ab/adduction angle, relative to the positive y-axis
    abduction_angle = phi + hip_foot_angle

    # theta: Angle between the tilted negative z-axis and the hip-to-foot
    # vector
    theta = np.arctan2(-x, R_hip_foot_yz)

    # Distance between the hip and foot
    R_hip_foot = (R_hip_foot_yz ** 2 + x ** 2) ** 0.5

    # Angle between the line going from hip to foot and the link L1
    arccos_argument = (config.LEG_L1 ** 2 + R_hip_foot ** 2 -
                       config.LEG_L2 ** 2) / (2 * config.LEG_L1 * R_hip_foot)
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    trident = np.arccos(arccos_argument)

    # Angle of the first link relative to the tilted negative z axis
    hip_angle = theta + trident

    # Angle between the leg links L1 and L2
    arccos_argument = (config.LEG_L1 ** 2 + config.LEG_L2 ** 2 -
                       R_hip_foot ** 2) / (2 * config.LEG_L1 * config.LEG_L2)
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    beta = np.arccos(arccos_argument)

    # Angle of the second link relative to the tilted negative z axis
    knee_angle = hip_angle - (np.pi - beta)

    return np.array([abduction_angle, hip_angle, knee_angle])


def four_legs_inverse_kinematics(r_body_foot, config):
    """
    Find all twelve joint angles for a set of body-relative foot positions.

    Parameters
    ----------
    r_body_foot : array_like
        Matrix of foot positions in the body frame, shape (3, 4).
    config : Config
        Robot configuration parameters.

    Returns
    -------
    numpy.ndarray
        Joint angles matrix, shape (3, 4).

    """
    alpha = np.zeros((3, 4))
    for i in range(4):
        body_offset = config.LEG_ORIGINS[:, i]
        alpha[:, i] = leg_explicit_inverse_kinematics(
            r_body_foot[:, i] - body_offset, i, config
        )
    return alpha
