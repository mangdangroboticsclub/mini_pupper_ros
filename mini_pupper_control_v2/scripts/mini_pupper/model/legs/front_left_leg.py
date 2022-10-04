from decimal import Decimal
from .leg import Leg


class FrontLeftLeg(Leg):

    @property
    def hip_angle(self) -> Decimal:
        return self.hip_initial_angle + self.hip_joint_position

    @property
    def thigh_angle(self) -> Decimal:
        return self.thigh_initial_angle + self.thigh_joint_position

    @property
    def calf_angle(self) -> Decimal:
        return self.calf_initial_angle + Decimal(90.0) + self.thigh_joint_position + self.calf_joint_position
