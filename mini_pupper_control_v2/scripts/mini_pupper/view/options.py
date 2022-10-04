"""Functions related to 'options' for the calibration tool."""
from rich import box
from rich.align import Align
from rich.panel import Panel
from rich.table import Table


def create_options_panel() -> Panel:
    """Create an options rich.Panel that relays user options."""
    options = Table.grid(padding=1)
    options.add_column()
    options.add_column()
    options.add_column()
    options.add_column()

    options.add_row('q/Q: Quit', 'a/A: Apply', '1-4: Select Leg', '', 'h/H: Select Hip')
    options.add_row('', 'i/I: Increase', '', '', 't/T: Select Thigh')
    options.add_row('', 'd/D: Decrease', '', '', 'c/C: Select Calf')

    return Panel(
        Align.center(options, vertical='middle'),
        box=box.ROUNDED,
        title='Keyboard Options'
    )
