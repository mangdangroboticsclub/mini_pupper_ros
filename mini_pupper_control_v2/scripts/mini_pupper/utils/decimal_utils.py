from decimal import ROUND_HALF_EVEN, Decimal
from enum import Enum
from typing import Union


class DecimalPoint(Decimal, Enum):
    FIVE: Decimal = Decimal('.00001')
    ZERO: Decimal = Decimal('1.')


def round_decimal(value: Union[float, int, Decimal], decimal_points: Decimal = DecimalPoint.FIVE) -> Decimal:
    return Decimal(value).quantize(decimal_points, rounding=ROUND_HALF_EVEN)
