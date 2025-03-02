import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from MangDang.mini_pupper.Config import Configuration
from .MovementScheme import MovementScheme
from .createDanceActionListSample import MovementLib
from mini_pupper_interfaces.msg import Command


class MiniPupperDanceNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_dance_node')

        # Create config
        self.config = Configuration()

        # Create movement group scheme instance and set a default True state
        self.movementCtl = MovementScheme(MovementLib)
        self.lib_length = len(MovementLib)

        self.commands_publisher = self.create_publisher(Command, 'robot_command', 10)

        # Create a timer to run the main loop at the desired frequency, 
        self.timer = self.create_timer(self.config.dt, self.main_loop)

    def main_loop(self):
        # Calculate legsLocation, attitudes, and speed using custom movement script
        self.movementCtl.runMovementScheme()
        command = Command()
        command.pseudo_dance_event = True
        command.legslocation = self.movementCtl.getMovemenLegsLocation()
        command.horizontal_velocity = self.movementCtl.getMovemenSpeed()
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
