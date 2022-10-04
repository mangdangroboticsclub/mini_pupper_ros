from __future__ import annotations

from decimal import Decimal
from multiprocessing import Event
from os import system
from typing import List
from pydantic import BaseModel, Field

from .legs.front_left_leg import FrontLeftLeg
from .legs.front_right_leg import FrontRightLeg
from .legs.back_left_leg import BackLeftLeg
from .legs.back_right_leg import BackRightLeg
from ..utils import run_in_parallel


class MiniPupper(BaseModel):

    front_left_leg: FrontLeftLeg
    front_right_leg: FrontRightLeg
    back_left_leg: BackLeftLeg
    back_right_leg: BackRightLeg

    control_pins: List[int] = Field(default=[21, 25], const=True)

    def move(self, joint_positions: List[Decimal], initial_pose: Event) -> MiniPupper:
        if len(joint_positions) != 12:
            raise ValueError(
                '12 joint positions expected for movement'
            )

        self.front_left_leg.joint_positions(joint_positions[0:3])
        self.front_right_leg.joint_positions(joint_positions[3:6])
        self.back_left_leg.joint_positions(joint_positions[6:9])
        self.back_right_leg.joint_positions(joint_positions[9:12])

        if not initial_pose.is_set():
            self.stand(initial_pose)
        else:
            self.front_left_leg.move(initial_pose)
            self.front_right_leg.move(initial_pose)
            self.back_left_leg.move(initial_pose)
            self.back_right_leg.move(initial_pose)

        return self

    def stand(self, initial_pose: Event) -> MiniPupper:
        run_in_parallel([
            (self.front_left_leg.move, initial_pose),
            (self.front_right_leg.move, initial_pose),
            (self.back_left_leg.move, initial_pose),
            (self.back_right_leg.move, initial_pose)
        ])
        initial_pose.set()
        return self

    def sit(self) -> MiniPupper:
        run_in_parallel([
            (self.front_left_leg.reset,),
            (self.front_right_leg.reset,),
            (self.back_left_leg.reset,),
            (self.back_right_leg.reset,)
        ])
        return self

    def save_legs_positions(self) -> MiniPupper:
        self.front_left_leg.save_position()
        self.front_right_leg.save_position()
        self.back_left_leg.save_position()
        self.back_right_leg.save_position()
        return self

    def activate(self) -> MiniPupper:
        return self.__update_pins(True)

    def deactivate(self) -> MiniPupper:
        return self.__update_pins(False)

    def __update_pins(self, should_activate: bool) -> MiniPupper:
        for pin in self.control_pins:
            system(f"echo {int(should_activate)} > /sys/class/gpio/gpio{str(pin)}/value")
        return self
