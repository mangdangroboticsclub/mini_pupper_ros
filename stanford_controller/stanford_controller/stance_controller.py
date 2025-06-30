import numpy as np
from transforms3d.euler import euler2mat


class StanceController:
    def __init__(self, config):
        self.config = config

    def position_delta(self, leg_index, state, command):
        """
        Calculate the positional and rotational delta for a foot in stance phase.

        Parameters
        ----------
        leg_index : int
            Index of the leg (0-based).
        state : State
            Current robot state containing foot locations and height.
        command : Command
            Command message specifying horizontal_velocity and yaw_rate.

        Returns
        -------
        tuple of (numpy.ndarray, numpy.ndarray)
            Position increment (shape (3,)) and rotation matrix increment (shape (3, 3)).

        """
        z = state.foot_locations[2, leg_index]
        v_xy = np.array(
            [
                -command.horizontal_velocity[0],
                -command.horizontal_velocity[1],
                1.0
                / self.config.z_time_constant
                * (state.height - z),
            ]
        )
        delta_p = v_xy * self.config.dt
        delta_R = euler2mat(0, 0, -command.yaw_rate * self.config.dt)
        return (delta_p, delta_R)

    # TODO: put current foot location into state
    def next_foot_location(self, leg_index, state, command):
        foot_location = state.foot_locations[:, leg_index]
        (delta_p, delta_R) = self.position_delta(leg_index, state, command)
        incremented_location = delta_R @ foot_location + delta_p

        return incremented_location
