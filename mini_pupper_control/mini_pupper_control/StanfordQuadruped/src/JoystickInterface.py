# Copyright 2023 stanfordroboticsclub
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import UDPComms
import numpy as np
from src.State import BehaviorState
from src.Command import Command
from src.Utilities import deadband, clipped_first_order_filter
from MangDang.mini_pupper.shutdown import ShutDown


class JoystickInterface:
    def __init__(
        self, config, udp_port=8830, udp_publisher_port=8840,
    ):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_activate_toggle = 0

        self.message_rate = 50
        self.udp_handle = UDPComms.Subscriber(udp_port, timeout=0.3)
        self.udp_publisher = UDPComms.Publisher(udp_publisher_port)

        self.sutdown_time = ShutDown()

    def get_command(self, state, disp, do_print=False):
        try:
            msg = self.udp_handle.get()
            command = Command()

            # Check for shotdown requests
            if msg["triangle"]:
                disp.show_state(BehaviorState.SHUTDOWN)
                self.sutdown_time.request_shutdown()
            else:
                self.sutdown_time.cancel_shutdown()

            # Check if requesting a state transition to trotting,
            # or from trotting to resting
            gait_toggle = msg["R1"]
            command.trot_event = (
                gait_toggle == 1 and self.previous_gait_toggle == 0)

            # Check if requesting a state transition to hopping,
            # from trotting or resting
            hop_toggle = msg["x"]
            command.hop_event = (
                hop_toggle == 1 and self.previous_hop_toggle == 0)

            activate_toggle = msg["L1"]
            command.activate_event = (
                activate_toggle == 1 and self.previous_activate_toggle == 0)

            # Update previous values for toggles and state
            self.previous_gait_toggle = gait_toggle
            self.previous_hop_toggle = hop_toggle
            self.previous_activate_toggle = activate_toggle

            x_vel = msg["ly"] * self.config.max_x_velocity
            y_vel = msg["lx"] * -self.config.max_y_velocity
            command.horizontal_velocity = np.array([x_vel, y_vel])
            command.yaw_rate = msg["rx"] * -self.config.max_yaw_rate

            message_rate = msg["message_rate"]
            message_dt = 1.0 / message_rate

            pitch = msg["ry"] * self.config.max_pitch
            deadbanded_pitch = deadband(
                pitch, self.config.pitch_deadband
            )
            pitch_rate = clipped_first_order_filter(
                state.pitch,
                deadbanded_pitch,
                self.config.max_pitch_rate,
                self.config.pitch_time_constant,
            )
            command.pitch = state.pitch + message_dt * pitch_rate

            height_movement = msg["dpady"]
            command.height = state.height - message_dt * \
                self.config.z_speed * height_movement

            roll_movement = - msg["dpadx"]
            command.roll = state.roll + \
                message_dt * self.config.roll_speed * roll_movement

            return command

        except UDPComms.timeout:
            if do_print:
                print("UDP Timed out")
            return Command()

    def set_color(self, color):
        joystick_msg = {"ps4_color": color}
        self.udp_publisher.send(joystick_msg)
