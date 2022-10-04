from decimal import Decimal
from math import pi
from pytest import mark

from ..utils import round_decimal, DecimalPoint, to_valid_angle, to_micros, to_degrees, to_valid_duty_cycle


class TestServoUtils:

    @mark.parametrize('actual_angle, expected_angle', [
        (Decimal('inf'), Decimal(180)),
        (Decimal('nan'), Decimal(180)),
        (Decimal(0), Decimal(0)),
        (Decimal(180), Decimal(180)),
        (Decimal(-1), Decimal(1)),
        (Decimal(181), Decimal(1)),
    ])
    def test_should_fix_invalid_angle(self, actual_angle: Decimal, expected_angle: Decimal):
        assert to_valid_angle(actual_angle) == expected_angle

    @mark.parametrize('actual_angle, expected_angle', [
        (Decimal('inf'), Decimal(180)),
        (Decimal('nan'), Decimal(180)),
        (Decimal(0), Decimal(0)),
        (Decimal(pi), Decimal(180)),
        (Decimal(-0.0174533), Decimal(-1)),
        (Decimal(3.15905), Decimal(181))
    ])
    def test_should_convert_to_degrees(self, actual_angle: Decimal, expected_angle: Decimal):
        assert round_decimal(to_degrees(actual_angle), DecimalPoint.ZERO) == expected_angle

    @mark.parametrize('actual_duty_cycle, expected_duty_cycle', [
        (-1, 500),
        (499, 500),
        (500, 500),
        (2500, 2500),
        (2501, 2500),
    ])
    def test_should_fix_invalid_duty_cycle(self, actual_duty_cycle: int, expected_duty_cycle: int):
        assert to_valid_duty_cycle(actual_duty_cycle) == expected_duty_cycle

    @mark.parametrize('actual_duty_cycle, expected_duty_cycle', [
        (500, 500000),
        (1500, 1500000),
        (2500, 2500000)
    ])
    def test_should_convert_to_micros(self, actual_duty_cycle: int, expected_duty_cycle: int):
        assert to_micros(actual_duty_cycle) == expected_duty_cycle
