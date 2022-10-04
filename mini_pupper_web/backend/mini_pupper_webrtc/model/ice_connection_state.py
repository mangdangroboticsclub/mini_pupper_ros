from enum import Enum


class IceConnectionState(str, Enum):
    FAILED = 'failed'
    CLOSED = 'closed'
    CHECKING ='checking'
    COMPLETED = 'completed'
