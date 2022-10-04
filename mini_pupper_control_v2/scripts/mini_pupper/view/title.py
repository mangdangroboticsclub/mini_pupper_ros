"""Functions related to 'title' for the calibration tool."""
from rich import box
from rich.align import Align
from rich.panel import Panel


def create_title_panel() -> Panel:
    """Create an options rich.Panel that relays user options."""
    title = '[b red]Mini Pupper CLI Calibration Tool[/b red]'
    return Panel(Align.center(title, vertical='middle'), box=box.ROUNDED)
