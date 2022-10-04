from __future__ import annotations
import time
from decimal import Decimal

from pydantic import BaseModel, root_validator, Field, PositiveFloat
from ...utils import round_decimal, to_micros, DecimalPoint
from ...utils.servo_utils import to_valid_angle, max_angle, min_angle, duty_cycle_180, duty_cycle_0


class ServoMotor(BaseModel):
    """
    Mangdang servo motors have the following duty cycle to angle mapping:
    500 ms -> 0 degrees
    1500 ms -> 90 degrees
    2500 ms -> 180 degrees
    To make an accurate offset depending on the current position we need to compute a slope
    as a diff between duty cycle boundaries divided by angles diff: (2500 - 500) / (180 - 0) = 11.11.
    To operate on microsecods level, we need to multiply the result by 1k
    """

    pin: int = Field(..., ge=1, le=40)
    initial_angle: Decimal = Field(default=Decimal(90), ge=0, le=180)
    current_angle: Decimal = Field(default=Decimal(0), ge=0, le=180)
    range_step_interval: PositiveFloat = Field(default=0.015, const=True)

    @property
    def slope(self) -> Decimal:
        return round_decimal(to_micros(duty_cycle_180 - duty_cycle_0) / (max_angle - min_angle))

    @root_validator
    def adjust_current_angle(cls, values: dict):
        values['current_angle'] = values.get('initial_angle', Decimal(90))
        return values

    def to_duty_cycle(self, angle: Decimal) -> Decimal:
        """
        Convert angle in degrees to servo motor's duty cycle.
        Note that the angle is adjusted based on [0; 180] range limit.
        """
        return round_decimal(to_micros(duty_cycle_0) + self.slope * to_valid_angle(angle), DecimalPoint.ZERO)

    def change_duty_cycle(self, value: Decimal) -> ServoMotor:
        """
        Write a new duty cycle value to corresponding pwm location.
        """
        with open(f"/sys/class/pwm/pwmchip0/pwm{self.pin}/duty_cycle", "w") as pwm:
            pwm.write(str(int(value)))
        return self

    def swap_initial_position(self) -> ServoMotor:
        """
        Update the initial position. Use it to save new calibration data.
        """
        self.initial_angle = self.current_angle
        return self

    def reset(self) -> ServoMotor:
        """
        Smooth move to the initial position
        """
        return self.range_move(self.current_angle, self.initial_angle, self.current_angle < self.initial_angle)

    def move(self, angle: Decimal, is_smooth: bool = True) -> ServoMotor:
        """
        Either move instantly or smoothly to the required angle
        """
        return self.range_move(self.initial_angle, angle, angle >= self.initial_angle) \
            if is_smooth else self.instant_move(angle)

    def instant_move(self, angle: Decimal) -> ServoMotor:
        """
        Instant change of the current angle
        """
        valid_angle = to_valid_angle(angle)
        duty_cycle = self.to_duty_cycle(valid_angle)

        self.change_duty_cycle(duty_cycle)
        self.current_angle = valid_angle

        return self

    def range_move(self, angle_from: Decimal, angle_to: Decimal, step_forward: bool = True) -> ServoMotor:
        """
        Smooth move in specified angles' range
        """
        valid_angle_from = to_valid_angle(angle_from)
        valid_angle_to = to_valid_angle(angle_to)
        step: int = 1 if step_forward else -1

        for angle in range(int(valid_angle_from), int(valid_angle_to) + step, step):
            self.instant_move(Decimal(angle))
            time.sleep(self.range_step_interval)

        return self

    def dict(self, *args, **kwargs):
        return {'pin': self.pin, 'initial_angle': int(self.initial_angle)}
