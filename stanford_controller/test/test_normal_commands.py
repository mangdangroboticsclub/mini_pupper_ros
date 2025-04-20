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


class TestNormalCommands(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_normal_commands')
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
        """Test sending normal commands and verify received states."""
        expected_states = self.load_expected_states('expected_normal_states.txt')
        total_num_of_commands = 101

        # Now start sending commands
        for command_counter in range(total_num_of_commands):
            if command_counter == 0:
                command = self.get_command(horizontal_velocity=[0.1, 0.0], trot_event=True)
            else:
                command = self.get_command(horizontal_velocity=[0.1, 0.0], trot_event=False)
            self.command_pub.publish(command)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            print(f"Sent command {command_counter}")

            # Wait for 1 second between commands
            time.sleep(1)

        assert len(self.received_states) == total_num_of_commands, \
            f"Expected 11 states, but received {len(self.received_states)}."

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

    def get_command(self, horizontal_velocity, trot_event):
        """Create a Command message with specified horizontal velocity and trot event."""
        command = Command()
        command.horizontal_velocity = horizontal_velocity
        matrix = Matrix3x4()
        matrix.row1 = [0.06, 0.06, -0.06, -0.06]
        matrix.row2 = [-0.05, 0.05, -0.05, 0.05]
        matrix.row3 = [-0.07, -0.07, -0.07, -0.07]
        command.foot_location = matrix
        command.height = -0.07
        command.trot_event = trot_event
        return command
