import logging

from aiortc import RTCDataChannel
from pydantic import ValidationError

from .uvicorn_logger import UvicornLogger
from .event_type import EventType
from ..dto.data_channel_payload import DataChannelPayload
from ..dto.data_channel_message import DataChannelMessage
from ..dto.data_channel_message_type import DataChannelMessageType


class DataChannel:

    def __init__(self, channel: RTCDataChannel):
        self.channel = channel
        self.channel.on(EventType.MESSAGE, self.on_message)
        self.logger = logging.getLogger(UvicornLogger.ERROR)

    def send(self, message: DataChannelMessage):
        if self.channel:
            self.channel.send(message.json())

    def close(self):
        if self.channel:
            self.channel.close()

    def on_message(self, data: str):
        try:
            message = DataChannelMessage.parse_raw(data)
            message_type = message.type.upper()

            if message_type == DataChannelMessageType.PING:
                self.send(DataChannelMessage(type=DataChannelMessageType.PONG))
            elif message_type == DataChannelMessageType.STREAM_CLOSED:
                self.send(DataChannelMessage(
                    type=DataChannelMessageType.CLOSED_SUCCESSFUL,
                    payload=DataChannelPayload(message="Closing the channel")
                ))
                self.close()
            else:
                self.send(DataChannelMessage(
                    type=DataChannelMessageType.BAD_REQUEST,
                    payload=DataChannelPayload(
                        message=f"Unsupported action: {message_type}",
                        received=message
                    )
                ))
        except ValidationError as error:
            self.send(DataChannelMessage(
                type=DataChannelMessageType.BAD_REQUEST,
                payload=DataChannelPayload(
                    message="Invalid data",
                    received=message,
                    error=error.json()
                )
            ))
        except Exception as error:
            self.logger.error(error)
            self.send(DataChannelMessage(
                type=DataChannelMessageType.SERVER_ERROR,
                payload=DataChannelPayload(message="Something's wrong on the server side")
            ))
            self.close()
