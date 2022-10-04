from pydantic import BaseModel


class WebRTCResponse(BaseModel):
    sdp: str = ''
    type: str = ''
