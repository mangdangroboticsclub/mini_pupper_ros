from pydantic import BaseModel

from .minipupper import MiniPupper


class CalibrationData(BaseModel):
    minipupper: MiniPupper

    def snapshot(self):
        self.minipupper.save_legs_positions()
        return self.dict(skip_defaults=True)
