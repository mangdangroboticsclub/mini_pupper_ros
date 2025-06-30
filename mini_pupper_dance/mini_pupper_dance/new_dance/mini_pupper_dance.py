#
# Copyright 2025 MangDang (www.mangdang.net)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import rclpy
from rclpy.node import Node
from stanford_controller.Config import Configuration
from .MovementScheme import MovementScheme
from .createDanceActionListSample import MovementLib
from mini_pupper_interfaces.msg import Command
from mini_pupper_interfaces.msg import Matrix3x4


class MiniPupperDanceNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_dance_node')

        # Create config
        self.config = Configuration()

        # Create movement group scheme instance and set a default True state
        self.movementCtl = MovementScheme(MovementLib)
        self.lib_length = len(MovementLib)

        self.commands_publisher = self.create_publisher(Command, 'robot_command', 10)

        # Create a timer to run the main loop at the desired frequency
        self.timer = self.create_timer(self.config.dt, self.main_loop)

    def main_loop(self):
        # Calculate legsLocation, attitudes, and speed using custom movement script
        self.movementCtl.runMovementScheme()
        command = Command()
        command.height = -0.07
        command.pseudo_dance_event = True
        command.horizontal_velocity = self.movementCtl.getMovemenSpeed()

        legsLocation = self.movementCtl.getMovemenLegsLocation()
        matrix = Matrix3x4()
        matrix.row1 = legsLocation[0]
        matrix.row2 = legsLocation[1]
        matrix.row3 = legsLocation[2]
        command.legs_location = matrix

        command.roll = self.movementCtl.attitude_now[0]
        command.pitch = self.movementCtl.attitude_now[1]
        command.yaw = self.movementCtl.attitude_now[2]
        command.yaw_rate = self.movementCtl.getMovemenTurn()
        self.commands_publisher.publish(command)

        # Check if the dance sequence is complete
        if (
            self.movementCtl.movement_now_number >= self.lib_length - 1
            and self.movementCtl.tick >= self.movementCtl.now_ticks
        ):
            self.get_logger().info("Dance sequence complete. Exiting...")
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    node = MiniPupperDanceNode()
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
