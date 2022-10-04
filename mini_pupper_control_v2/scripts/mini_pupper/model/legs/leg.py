from __future__ import annotations

from decimal import Decimal
from multiprocessing import Event
from typing import List, Union

from pydantic import BaseModel, Field

from ...utils import run_in_parallel, to_degrees
from ..servos.hip import Hip
from ..servos.calf import Calf
from ..servos.thigh import Thigh


class Leg(BaseModel):
    hip: Hip
    thigh: Thigh
    calf: Calf

    hip_joint_position: Decimal = Field(default=Decimal(0.0), ge=0, le=180)
    thigh_joint_position: Decimal = Field(default=Decimal(0.0), ge=0, le=180)
    calf_joint_position: Decimal = Field(default=Decimal(0.0), ge=0, le=180)

    def joint_positions(self, joint_positions: List[Union[int, float, Decimal]]) -> Leg:
        if len(joint_positions) != 3:
            raise ValueError(
                'Leg position must be a list of 3 values: hip, thigh, calf'
            )

        self.hip_joint_position = to_degrees(joint_positions[0])
        self.thigh_joint_position = to_degrees(joint_positions[1])
        self.calf_joint_position = to_degrees(joint_positions[2])

        return self

    def move(self, initial_pose: Event) -> Leg:
        if not initial_pose.is_set():
            run_in_parallel([
                (self.move_hip, self.hip_angle, True),
                (self.move_thigh, self.thigh_angle, True),
                (self.move_calf, self.calf_angle, True)
            ])
        else:
            self.move_hip(self.hip_angle) \
                .move_thigh(self.thigh_angle) \
                .move_calf(self.calf_angle)
        return self

    def reset(self) -> Leg:
        run_in_parallel([(self.hip.reset,), (self.thigh.reset,), (self.calf.reset,)])
        return self

    def move_hip(self, angle: Decimal, should_slow_down: bool = False) -> Leg:
        self.hip.move(angle, should_slow_down)
        return self

    def move_thigh(self, angle: Decimal, should_slow_down: bool = False) -> Leg:
        self.thigh.move(angle, should_slow_down)
        return self

    def move_calf(self, angle: Decimal, should_slow_down: bool = False) -> Leg:
        self.calf.move(angle, should_slow_down)
        return self

    @property
    def hip_initial_angle(self) -> Decimal:
        return self.hip.initial_angle

    @property
    def thigh_initial_angle(self) -> Decimal:
        return self.thigh.initial_angle

    @property
    def calf_initial_angle(self) -> Decimal:
        return self.calf.initial_angle

    @property
    def hip_current_angle(self):
        return self.hip.current_angle

    @property
    def thigh_current_angle(self):
        return self.thigh.current_angle

    @property
    def calf_current_angle(self):
        return self.calf.current_angle

    # These properties should be overridden in child classes
    @property
    def hip_angle(self) -> Decimal:
        return self.hip_initial_angle

    @property
    def thigh_angle(self) -> Decimal:
        return self.thigh_initial_angle

    @property
    def calf_angle(self) -> Decimal:
        return self.calf_initial_angle

    @property
    def name(self):
        return self.__class__.__name__

    def save_position(self):
        self.hip.swap_initial_position()
        self.thigh.swap_initial_position()
        self.calf.swap_initial_position()
