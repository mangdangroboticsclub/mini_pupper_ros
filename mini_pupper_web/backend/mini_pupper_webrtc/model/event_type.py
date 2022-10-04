from enum import Enum


class EventType(str, Enum):
    MESSAGE = 'message'
    ICE_CONNECTION_STATE_CHANGE = 'iceconnectionstatechange'
    TRACK = 'track'
    DATA_CHANNEL = 'datachannel'
