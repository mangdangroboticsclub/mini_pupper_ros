# flake8: noqa: E402  # Add this to suppress import position errors
import os
import sys  # Standard library imports

# Add the path to the mini_pupper_dance module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../mini_pupper_dance'))

import pytest
import rclpy
import time
from launch import LaunchDescription
import launch_ros.actions
import launch_testing
import unittest
from std_msgs.msg import String
from mini_pupper_interfaces.msg import Command, Matrix3x4
from mini_pupper_dance.MovementGroup import MovementGroups
from mini_pupper_dance.MovementScheme import MovementScheme


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
        """Test sending dance commands