from decimal import Decimal
from math import isnan, isinf, pi
from typing import Union

min_angle: Decimal = Decimal(0.0)
max_angle: Decimal = Decimal(180.0)
rad: Decimal = max_angle / Decimal(pi)
micros_koef: int = 1000
duty_cycle_0: int = 500
duty_cycle_180: int = 2500


def is_valid_decimal(angle: Decimal) -> bool:
    return not isnan(angle) and not isinf(angle)


def to_valid_angle(angle: Decimal) -> Decimal:
    valid_angle = angle if is_valid_decimal(angle) else max_angle
    limited_angle = abs(valid_angle) % max_angle
    return max_angle if limited_angle == min_angle and valid_angle != min_angle else limited_angle


def to_valid_duty_cycle(duty_cycle: int) -> int:
    return duty_cycle_0 if duty_cycle < duty_cycle_0 else \
        duty_cycle_180 if duty_cycle > duty_cycle_180 else duty_cycle


def to_micros(duty_cycle: int) -> Decimal:
    return Decimal(micros_koef * to_valid_duty_cycle(duty_cycle))


def to_degrees(joint_position: Union[int, float, Decimal]) -> Decimal:
    return Decimal(joint_position) * rad if is_valid_decimal(joint_position) else max_angle
