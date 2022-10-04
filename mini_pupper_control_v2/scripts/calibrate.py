"""Mini-Pupper non-GUI Calibration Tool"""
import sys
import termios

from rich import print as r_print
from rich.layout import Layout

from mini_pupper.cli.keyboard import get_key
from mini_pupper.view.options import create_options_panel
from mini_pupper.view.title import create_title_panel
from mini_pupper.view.pupper import Pupper

calibration_file_path = 'mini_pupper/calibration.yaml'


def main():
    """Run the mini pupper calibration tool."""
    settings = termios.tcgetattr(sys.stdin)
    pupper = Pupper(calibration_file_path)
    leg_options = {
        '1': pupper.front_left_leg.name,
        '2': pupper.front_right_leg.name,
        '3': pupper.back_left_leg.name,
        '4': pupper.back_right_leg.name,
    }

    # Create layout containing the minipupper leg and joint selection
    layout = Layout()
    layout.split_column(
        Layout(size=2),
        Layout(name='title_bar')
    )
    layout['title_bar'].split_column(
        Layout(create_title_panel(), name='title', size=5),
        Layout(name='upper')
    )
    layout['upper'].split_column(
        Layout(name='front_legs_viz', size=10),
        Layout(name='lower')
    )
    layout['lower'].split_column(
        Layout(name='back_legs_viz', size=10),
        Layout(create_options_panel(), name='options', size=10),
    )
    layout['front_legs_viz'].split_row(
        Layout(pupper.front_left_leg.update(True), name=pupper.front_left_leg.name),
        Layout(pupper.front_right_leg.update(), name=pupper.front_right_leg.name),
    )
    layout['back_legs_viz'].split_row(
        Layout(pupper.back_left_leg.update(), name=pupper.back_left_leg.name),
        Layout(pupper.back_right_leg.update(), name=pupper.back_right_leg.name),
    )

    # Print initial layout
    r_print(layout)

    # Select default leg and joint
    selected_leg = pupper.front_left_leg.name
    joint_selection = 'h'

    # Run the calibration tool
    while True:
        keyboard_input = get_key(settings).lower()
        if keyboard_input == 'q':
            break

        if keyboard_input == 'a':
            pupper.save_calibration_data()
            continue

        if keyboard_input in map(lambda i: str(i), range(1, 5)):
            for key, leg_name in leg_options.items():
                is_leg_selected = key == keyboard_input
                if is_leg_selected:
                    selected_leg = leg_name

                layout[leg_name].update(
                    pupper.leg(leg_name).update(is_leg_selected)
                )

            r_print(layout, end='\r')
        elif keyboard_input in ['h', 't', 'c']:
            joint_selection = keyboard_input.lower()
            r_print(layout, end='\r')

        elif keyboard_input in ['i', 'd']:
            pupper.leg(selected_leg).change_angle(joint_selection, keyboard_input.lower())
            layout[selected_leg].update(
                pupper.leg(selected_leg).update(True)
            )
            r_print(layout, end='\r')


if __name__ == '__main__':
    main()
