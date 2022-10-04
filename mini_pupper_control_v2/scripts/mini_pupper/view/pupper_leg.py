from typing import Callable

from pydantic import BaseModel
from rich import box
from rich.align import Align
from rich.panel import Panel
from rich.table import Table

from ..model.legs.leg import Leg
from ..utils.servo_utils import min_angle, max_angle


class PupperLeg(BaseModel):
    leg: Leg
    color: str

    @property
    def name(self) -> str:
        return self.leg.name

    @property
    def hip(self) -> int:
        return int(self.leg.hip_current_angle)

    @property
    def thigh(self) -> int:
        return int(self.leg.thigh_current_angle)

    @property
    def calf(self) -> int:
        return int(self.leg.calf_current_angle)

    @property
    def min_angle(self):
        return int(min_angle)

    @property
    def max_angle(self):
        return int(max_angle)

    def generate_table(self) -> Table:
        """Generate rich.Table with current hip, calf, and thigh values."""
        table = Table()
        table.add_column(f'{self.name}', justify='right', style='cyan')
        table.add_column('Value', style='magenta')

        table.add_row('Hip', f'{self.min_angle} <---------- {self.hip} ----------> {self.max_angle}')
        table.add_row('Thigh', f'{self.min_angle} <---------- {self.thigh} ----------> {self.max_angle}')
        table.add_row('Calf', f'{self.min_angle} <---------- {self.calf} ----------> {self.max_angle}')

        return table

    def update(self, is_selected: bool = False) -> Panel:
        """Update leg information in the form of a rich.Panel."""
        table = self.generate_table()
        color = f'on {self.color}' if is_selected else self.color

        return Panel(
            Align.center(table, vertical='middle'),
            title=self.name,
            box=box.ROUNDED,
            style=color
        )

    def change_angle(self, joint: str, action: str) -> None:
        update_angle: Callable[[int], int] = lambda angle: angle + 1 \
            if action == 'i' else angle - 1 if action == 'd' else angle

        if joint == 'h':
            self.leg.move_hip(update_angle(self.hip))
        elif joint == 't':
            self.leg.move_thigh(update_angle(self.thigh))
        elif joint == 'c':
            self.leg.move_calf(update_angle(self.calf))
