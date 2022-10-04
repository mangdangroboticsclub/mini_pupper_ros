from ..model.options import Options
from pydantic import BaseModel


class OfferPayload(BaseModel):
    options: Options
    sdp: str
    type: str
