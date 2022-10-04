"""Pupper class definition."""
from yaml import safe_load, dump

from ..model.calibration_data import CalibrationData
from ..utils import camel_to_snake
from ..view.pupper_leg import PupperLeg
from ..model import MiniPupper


class Pupper:
    """MiniPupper Class containing joint values and other attributes."""

    def __init__(self, calibration_file: str):
        self.calibration_file = calibration_file
        self.minipupper: MiniPupper = self.load_calibration_data().minipupper
        self.front_left_leg = PupperLeg(leg=self.minipupper.front_left_leg, color='green')
        self.front_right_leg = PupperLeg(leg=self.minipupper.front_right_leg, color='blue')
        self.back_left_leg = PupperLeg(leg=self.minipupper.back_left_leg, color='green')
        self.back_right_leg = PupperLeg(leg=self.minipupper.back_right_leg, color='blue')

    def leg(self, name) -> PupperLeg:
        return self.__dict__[camel_to_snake(name)]

    def load_calibration_data(self) -> CalibrationData:
        with open(self.calibration_file, "r") as f:
            calibration_data = CalibrationData.parse_obj(safe_load(f))
        return calibration_data

    def save_calibration_data(self) -> None:
        legs = MiniPupper(
            front_left=self.front_left_leg.leg,
            front_right=self.front_right_leg.leg,
            back_left=self.back_left_leg.leg,
            back_right=self.back_right_leg.leg
        )
        calibration_data = CalibrationData(legs=legs)
        with open(self.calibration_file, "w") as f:
            dump(calibration_data.snapshot(), f, default_flow_style=False)
