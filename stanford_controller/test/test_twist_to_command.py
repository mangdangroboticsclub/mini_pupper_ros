import time
import unittest

import pytest
import rclpy
from geometry_msgs.msg import Twist
from mini_pupper_interfaces.msg import Command
from launch import LaunchDescription
import launch_ros.actions
import launch_testing


@pytest.mark.rostest
def generate_test_description():
    twist_node = launch_ros.actions.Node(
        package='stanford_controller',
        executable='twist_to_command_node',
        name='twist_to_command_node',
    )
    return LaunchDescription([
        twist_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestTwistToCommandNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # create a test node, publisher to cmd_vel, subscriber to robot_command
        self.node = rclpy.create_node('test_twist_to_command')
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.received_cmds = []
        self.command_sub = self.node.create_subscription(
            Command,
            'robot_command',
            lambda msg: self.received_cmds.append(msg),
            10
        )

        # wait for connections
        timeout = time.time() + 5.0
        while time.time() < timeout and self.cmd_vel_pub.get_subscription_count() == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

    def tearDown(self):
        self.node.destroy_subscription(self.command_sub)
        self.node.destroy_node()

    def test_trot_event_sequence(self):
        # send 3 non-zero forward twists
        for _ in range(3):
            tw = Twist()
            tw.linear.x = 0.1
            tw.linear.y = 0.0
            tw.angular.z = 0.0
            self.cmd_vel_pub.publish(tw)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            # give the node a moment to process & republish
            time.sleep(0.1)

        # send 2 zero twists to stop the robot
        for _ in range(2):
            tw = Twist()
            tw.linear.x = 0.0
            tw.linear.y = 0.0
            tw.angular.z = 0.0
            self.cmd_vel_pub.publish(tw)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            # give the node a moment to process & republish
            time.sleep(0.1)

        # we expect 5 commands back
        self.assertEqual(len(self.received_cmds), 5)

        # first publish should set trot_event=1, next two trot_event=0
        expected_trot_events = [1, 0, 0]
        for i in range(3):
            trot_event = self.received_cmds[i].trot_event
            forward_velocity = self.received_cmds[i].horizontal_velocity[0]
            self.assertEqual(
                trot_event,
                expected_trot_events[i],
                f"message {i} trot_event was {trot_event}, expected {expected_trot_events[i]}"
            )
            self.assertEqual(
                forward_velocity,
                0.1,
                f"message {i} horizontal_velocity was {forward_velocity}, expected 0.1"
            )

        expected_trot_events = [1, 0]
        for i in range(2):
            trot_event = self.received_cmds[i+3].trot_event
            forward_velocity = self.received_cmds[i+3].horizontal_velocity[0]
            self.assertEqual(
                trot_event,
                expected_trot_events[i],
                f"message {i} trot_event was {trot_event}, expected {expected_trot_events[i]}"
            )
            self.assertEqual(
                forward_velocity,
                0.0,
                f"message {i} horizontal_velocity was {forward_velocity}, expected 0.0"
            )
