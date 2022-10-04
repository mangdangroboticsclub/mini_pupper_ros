from enum import Enum


class MedianFilter(str, Enum):
    MEDIAN_OFF = 'MEDIAN_OFF'
    KERNEL_3x3 = 'KERNEL_3x3'
    KERNEL_5x5 = 'KERNEL_5x5'
    KERNEL_7x7 = 'KERNEL_7x7'
