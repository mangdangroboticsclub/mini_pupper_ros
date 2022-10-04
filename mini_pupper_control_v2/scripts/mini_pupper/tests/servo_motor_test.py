from decimal import Decimal, ROUND_HALF_EVEN
from typing import Sequence

from mock.mock import call
from pytest_mock import MockerFixture
from pytest import fixture, mark
from random import randint

from ..model.servos.servo_motor import ServoMotor
from ..utils import round_decimal


class TestServoMotor:
    duty_cycle_range = [
        500000,
        511111,
        522222,
        533333,
        544444,
        555556,
        566667,
        577778,
        588889,
        600000,
        611111,
        622222,
        633333,
        644444,
        655556,
        666667,
        677778,
        688889,
        700000,
        711111,
        722222,
        733333,
        744444,
        755556,
        766667,
        777778,
        788889,
        800000,
        811111,
        822222,
        833333,
        844444,
        855556,
        866667,
        877778,
        888889,
        900000,
        911111,
        922222,
        933333,
        944444,
        955556,
        966667,
        977778,
        988889,
        1000000,
        1011111,
        1022222,
        1033333,
        1044444,
        1055556,
        1066667,
        1077778,
        1088889,
        1100000,
        1111111,
        1122222,
        1133333,
        1144444,
        1155556,
        1166667,
        1177778,
        1188889,
        1200000,
        1211111,
        1222222,
        1233333,
        1244444,
        1255556,
        1266667,
        1277778,
        1288889,
        1300000,
        1311111,
        1322222,
        1333333,
        1344444,
        1355556,
        1366667,
        1377778,
        1388889,
        1400000,
        1411111,
        1422222,
        1433333,
        1444444,
        1455556,
        1466667,
        1477778,
        1488889,
        1500000,
        1511111,
        1522222,
        1533333,
        1544444,
        1555556,
        1566667,
        1577778,
        1588889,
        1600000,
        1611111,
        1622222,
        1633333,
        1644444,
        1655556,
        1666667,
        1677778,
        1688889,
        1700000,
        1711111,
        1722222,
        1733333,
        1744444,
        1755556,
        1766667,
        1777778,
        1788889,
        1800000,
        1811111,
        1822222,
        1833333,
        1844444,
        1855556,
        1866667,
        1877778,
        1888889,
        1900000,
        1911111,
        1922222,
        1933333,
        1944444,
        1955556,
        1966667,
        1977778,
        1988889,
        2000000,
        2011111,
        2022222,
        2033333,
        2044444,
        2055556,
        2066667,
        2077778,
        2088889,
        2100000,
        2111111,
        2122222,
        2133333,
        2144444,
        2155556,
        2166667,
        2177778,
        2188889,
        2200000,
        2211111,
        2222222,
        2233333,
        2244444,
        2255556,
        2266667,
        2277778,
        2288889,
        2300000,
        2311111,
        2322222,
        2333333,
        2344444,
        2355556,
        2366667,
        2377778,
        2388889,
        2400000,
        2411111,
        2422222,
        2433333,
        2444444,
        2455556,
        2466667,
        2477778,
        2488889,
        2500000,
    ]

    angle_to_duty = ('angle, duty_cycle', [
        (-1, 511111),
        (0, 500000),
        (90, 1500000),
        (180, 2500000),
        (181, 511111),
        (360, 2500000)
    ])

    random_pin = randint(1, 40)
    min_angle = Decimal(0)
    neutral_angle = Decimal(90)
    max_angle = Decimal(180)
    min_duty_cycle = Decimal(500000)
    neutral_duty_cycle = Decimal(1500000)
    max_duty_cycle = Decimal(2500000)
    slope = Decimal(11111)

    @fixture
    def servo(self) -> ServoMotor:
        return ServoMotor(pin=self.random_pin)

    def test_current_angle_match_initial_angle(self, servo):
        assert servo.current_angle == servo.initial_angle == Decimal(90.0)

    @mark.parametrize('initial_angle', [0, 180])
    def test_initial_angle_validation(self, initial_angle):
        servo = ServoMotor(pin=self.random_pin, initial_angle=initial_angle)
        assert servo.current_angle == servo.initial_angle == initial_angle

    @mark.parametrize('pin', [1, 40])
    def test_pin_validation(self, pin):
        assert ServoMotor(pin=pin).pin == pin

    def test_slope_accuracy(self, servo):
        assert servo.slope == Decimal(11111.11111).quantize(Decimal('.00001'), rounding=ROUND_HALF_EVEN)

    @mark.parametrize('angle, duty_cycle', [
        (min_angle, min_duty_cycle),
        (neutral_angle, neutral_duty_cycle),
        (max_angle, max_duty_cycle),
        (max_angle + 1, min_duty_cycle + slope),
        (max_angle * 2, max_duty_cycle),
        (min_angle - Decimal(1.1), min_duty_cycle + 12222),
        (Decimal('inf'), max_duty_cycle),
        (Decimal('nan'), max_duty_cycle),
    ])
    def test_transformation_from_angle_to_duty_cycle(self, servo: ServoMotor, angle: Decimal, duty_cycle: Decimal):
        assert servo.to_duty_cycle(angle) == duty_cycle

    def test_swap_initial(self, servo):
        servo.current_angle = Decimal(120)
        servo.swap_initial_position()
        assert servo.initial_angle == servo.current_angle

    @mark.parametrize(*angle_to_duty)
    def test_instant_move(self, angle: int, duty_cycle: int, servo: ServoMotor, mocker: MockerFixture):
        change_duty_cycle_mock = mocker.patch.object(ServoMotor, 'change_duty_cycle')
        servo.instant_move(Decimal(angle))
        change_duty_cycle_mock.assert_called_once_with(Decimal(duty_cycle))

    @mark.parametrize('angle_from, angle_to, step_forward, duty_cycles', [
        (
            Decimal(45), Decimal(90), True,
            [call(Decimal(dc)) for angle, dc in enumerate(duty_cycle_range) if 45 <= angle <= 90]
        ),
        (
            Decimal(60), Decimal(0), False,
            [call(Decimal(dc)) for angle, dc in enumerate(duty_cycle_range) if 0 <= angle <= 60][::-1]
        ),
        (
            Decimal(-90), Decimal(90), True,
            [call(Decimal(dc)) for angle, dc in enumerate(duty_cycle_range) if angle == 90]
        ),
        (
            Decimal(0), Decimal(0), True,
            [call(Decimal(dc)) for angle, dc in enumerate(duty_cycle_range) if angle == 0]
        ),
        (
            Decimal(180), Decimal(180), True,
            [call(Decimal(dc)) for angle, dc in enumerate(duty_cycle_range) if angle == 180]
        ),
        (
            Decimal(-180), Decimal(0), False,
            [call(Decimal(dc)) for angle, dc in enumerate(duty_cycle_range) if 0 <= angle <= 180][::-1]
        ),
        (Decimal(180), Decimal(90), True, []),
        (Decimal(0), Decimal(180), False, [])
    ])
    def test_range_move(
        self,
        angle_from: Decimal,
        angle_to: Decimal,
        step_forward: bool,
        duty_cycles: Sequence,
        servo: ServoMotor,
        mocker: MockerFixture
    ):
        change_duty_cycle_mock = mocker.patch.object(ServoMotor, 'change_duty_cycle')
        servo.range_move(angle_from, angle_to, step_forward)
        change_duty_cycle_mock.assert_has_calls(duty_cycles)

    @mark.parametrize('initial_angle, current_angle, move_forward', [
        (Decimal(0), Decimal(180), False),
        (Decimal(180), Decimal(0), True),
    ])
    def test_should_reset_to_initial(
        self,
        initial_angle: Decimal,
        current_angle: Decimal,
        move_forward: bool,
        servo: ServoMotor,
        mocker: MockerFixture
    ):
        range_move_mock = mocker.patch.object(ServoMotor, 'range_move')
        servo.initial_angle = initial_angle
        servo.current_angle = current_angle
        servo.reset()
        range_move_mock.assert_has_calls([call(servo.current_angle, servo.initial_angle, move_forward)])

    @mark.parametrize('angle, is_smooth', [
        *[(Decimal(angle), is_smooth) for is_smooth in [True, False] for angle in [0, 90, 180]]
    ])
    def test_should_move(
        self,
        angle: Decimal,
        is_smooth: bool,
        servo: ServoMotor,
        mocker: MockerFixture
    ):
        move_mock = mocker.patch.object(ServoMotor, 'range_move' if is_smooth else 'instant_move')
        servo.move(angle, is_smooth)
        internal_call = call(servo.initial_angle, angle, angle >= servo.initial_angle) if is_smooth else call(angle)
        move_mock.assert_has_calls([internal_call])

    def __duty_cycle_in(self, angle_from, angle_to):
        return [call(Decimal(dc)) for angle, dc in enumerate(self.duty_cycle_range) if angle_from <= angle <= angle_to]
