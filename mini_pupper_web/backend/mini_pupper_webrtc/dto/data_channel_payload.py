from typing import Optional
from pydantic import BaseModel


class DataChannelPayload(BaseModel):
    message: str
    received: Optional[str]
    error: Optional[str]
