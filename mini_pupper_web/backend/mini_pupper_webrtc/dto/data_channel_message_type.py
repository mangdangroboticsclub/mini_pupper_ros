from enum import Enum


class DataChannelMessageType(str, Enum):
    PING = 'PING'
    PONG = 'PONG'
    STREAM_CLOSED = 'STREAM_CLOSED'
    CLOSED_SUCCESSFUL = 'CLOSED_SUCCESSFUL'
    BAD_REQUEST = 'BAD_REQUEST'
    SERVER_ERROR = 'SERVER_ERROR'
