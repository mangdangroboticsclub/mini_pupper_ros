import os
import pytest
import rclpy
import time
from launch import LaunchDescription
import launch_ros.actions
import launch_testing
import unittest
from std_msgs.msg import String

from mini_pupper_interfaces.msg import Command, Matrix3x4
from mini_pupper_dance.new_dance.MovementGroup import MovementGroups
from mini_pupper_dance.new_dance.MovementScheme import MovementScheme


@pytest.mark.rostest
def generate_test_description():
    controller_node = launch_ros.actions.Node(
        package='stanford_controller',
        executable='stanford_controller_node',
        name='stanford_controller_node',
        parameters=[{
            'orientation_from_imu': False,
            'publish_states': True
        }]
    )

    return LaunchDescription([
        controller_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestDanceCommands(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_dance_commands')
        self.command_pub = self.node.create_publisher(Command, 'robot_command', 10)
        self.received_states = []
        self.state_sub = self.node.create_subscription(
            String,
            'state_log',
            lambda msg: self.received_states.append(msg.data),
            10
        )
        time.sleep(3)  # Allow time for node to initialize

        # Ensure publisher has established connection with subscribers
        timeout = time.time() + 5.0  # 5 second timeout
        while time.time() < timeout and self.command_pub.get_subscription_count() == 0:
            print("Waiting for publisher connection...")
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.5)
        print(f"Publisher connected to {self.command_pub.get_subscription_count()} subscribers")

    def tearDown(self):
        self.node.destroy_subscription(self.state_sub)
        self.node.destroy_node()

    def test_send_commands(self):
        """Test sending dance commands and verify received states."""
        expected_states = self.load_expected_states('expected_dance_states.txt')
        total_num_of_commands = 142
        move_groups = MovementGroups()
        move_groups.move_forward()
        movementCtl = MovementScheme(move_groups.MovementLib)
        lib_length = len(move_groups.MovementLib)

        while True:
            movementCtl.runMovementScheme()
            command = Command()
            command.height = -0.07
            command.pseudo_dance_event = True

            legsLocation = movementCtl.getMovemenLegsLocation()
            matrix = Matrix3x4()
            matrix.row1 = legsLocation[0]
            matrix.row2 = legsLocation[1]
            matrix.row3 = legsLocation[2]
            command.foot_location = matrix
            command.attitude = movementCtl.getMovemenAttitude()
            command.robot_speed = movementCtl.getMovemenSpeed()
            self.command_pub.publish(command)
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # Check if the dance sequence is complete
            if (movementCtl.movement_now_number >= lib_length - 1
               and movementCtl.tick >= movementCtl.now_ticks):
                break

        assert len(self.received_states) == total_num_of_commands, \
            f"Expected {total_num_of_commands} states, but received {len(self.received_states)}."

        # Verify that the received states match the expected states
        for i, (r_state, e_state) in enumerate(zip(self.received_states, expected_states)):
            assert r_state.strip() == e_state.strip(), (
                f"State mismatch at iteration {i}:\n"
                f"Expected: {e_state}\n"
                f"Received: {r_state}"
            )

    def load_expected_states(self, file_name):
        """Load expected states from file."""
        test_dir = os.path.dirname(os.path.abspath(__file__))
        expected_states_file = os.path.join(test_dir, file_name)

        with open(expected_states_file, 'r') as f:
            content = f.read()

        # Split the content into states using double newlines as the delimiter
        expected_states = content.strip().split("\n\n")
        return expected_states
