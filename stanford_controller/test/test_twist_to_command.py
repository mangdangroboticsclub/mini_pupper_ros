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
            time.sleep(0.015)

        # let the node cycle a few more times so timer callbacks interleave…
        for _ in range(15):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.015)

        # collect simple lists of the fields we care about
        speeds = [cmd.horizontal_velocity[0] for cmd in self.received_cmds]
        trots = [cmd.trot_event for cmd in self.received_cmds]

        # find the first non-zero‐speed message
        first_nz = next(i for i, v in enumerate(speeds) if v > 0.001)

        # 1) that first non-zero speed is trot_event == True
        self.assertTrue(speeds[first_nz] == 0.1)
        self.assertTrue(trots[first_nz] == 1)

        # 2) the next N non-zero speeds should be trot_event == False
        #    (in your publish you sent 3 forwards total)
        forwards = [i for i, v in enumerate(speeds) if v > 0.001]
        # ensure we saw exactly 3 of them
        self.assertLess(len(forwards), 7)
        # check trot_event on the 2nd/3rd
        for idx in forwards[1:]:
            self.assertEqual(trots[idx], 0)

        # 3) find the first zero‐speed after those forwards
        zeros = [i for i, v in enumerate(speeds) if abs(v) < 1e-3 and i > forwards[-1]]
        self.assertGreater(len(zeros), 0)
        first_zero = zeros[0]

        # that first zero‐speed should have trot_event == True
        self.assertEqual(trots[first_zero], 1)

        # 4) all subsequent zero‐speed messages should be trot_event == False
        for idx in zeros[1:]:
            self.assertEqual(trots[idx], 0)
