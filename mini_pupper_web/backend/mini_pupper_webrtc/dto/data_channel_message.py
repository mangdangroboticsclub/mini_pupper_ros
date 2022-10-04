from typing import Optional
from pydantic import BaseModel
from .data_channel_payload import DataChannelPayload
from .data_channel_message_type import DataChannelMessageType


class DataChannelMessage(BaseModel):
    type: DataChannelMessageType
    payload: Optional[DataChannelPayload]
