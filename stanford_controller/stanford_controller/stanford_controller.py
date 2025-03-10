import rclpy
from rclpy.node import Node
from .gait_controller import GaitController
from .stance_controller import StanceController
from .swing_controller import SwingController

from .Kinematics import four_legs_inverse_kinematics
from .Utilities import clipped_first_order_filter
from .State import BehaviorState, State

from MangDang.mini_pupper.Config import Configuration

import numpy as np
from transforms3d.euler import euler2mat, quat2euler

from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from mini_pupper_interfaces.msg import Command
from mini_pupper_interfaces.msg import Matrix3x4

class StanfordController(Node):
    """
    ROS 2 Node for StanfordController
    """

    def __init__(self, config, inverse_kinematics):
        super().__init__('stanford_controller')

        # Read parameters
        self.declare_parameter('orientation_from_imu', False)
        self.orientation_from_imu = self.get_parameter('orientation_from_imu').get_parameter_value().bool_value
        self.get_logger().info(f"use_imu: {self.orientation_from_imu}")

        # Configuration and initialization
        self.joint_names = [
            "base_lf1", "lf1_lf2", "lf2_lf3",
            "base_rf1", "rf1_rf2", "rf2_rf3",
            "base_lb1", "lb1_lb2", "lb2_lb3",
            "base_rb1", "rb1_rb2", "rb2_rb3"
        ]
        self.config = config
        self.inverse_kinematics = inverse_kinematics
        self.smoothed_yaw = 0.0
        self.dance_active_state = False

        self.contact_modes = np.zeros(4)
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)

        self.hop_transition_mapping = {
            BehaviorState.REST: BehaviorState.HOP,
            BehaviorState.HOP: BehaviorState.FINISHHOP,
            BehaviorState.FINISHHOP: BehaviorState.REST,
            BehaviorState.TROT: BehaviorState.HOP,
        }
        self.trot_transition_mapping = {
            BehaviorState.REST: BehaviorState.TROT,
            BehaviorState.TROT: BehaviorState.REST,
            BehaviorState.HOP: BehaviorState.TROT,
            BehaviorState.FINISHHOP: BehaviorState.TROT,
        }
        self.activate_transition_mapping = {
            BehaviorState.DEACTIVATED: BehaviorState.REST,
            BehaviorState.REST: BehaviorState.DEACTIVATED,
        }

        # ROS 2 publishers and subscribers
        self.command_subscriber = self.create_subscription(
            Command,
            'robot_command',
            self.command_callback,
            10
        )

        if self.orientation_from_imu:
            self.imu_subscriber = self.create_subscription(
                Imu,
                'imu/data',
                self.imu_callback,
                10
            )

        self.joint_commands_publisher = self.create_publisher(
            JointTrajectory, 
            'joint_group_effort_controller/joint_trajectory',
            10
        )

        # Timer to run the control loop
        self.control_timer = self.create_timer(
            0.01, # 100 Hz
            self.control_loop_callback
        )

        self.state = State()  
        self.current_command = None  # Command message
        self.quat_orientation = np.array([1, 0, 0, 0])

    def command_callback(self, msg):
        """
        Callback to handle incoming Command messages.
        """
        self.get_logger().info(f'''command_callback, 
                               roll: {msg.roll}
                               pitch: {msg.pitch}
                               yaw: {msg.yaw}''')
        self.get_logger().info(f'''command_callback, foot_location
                               row0: {msg.foot_location.row1} 
                               row2: {msg.foot_location.row2} 
                               row3: {msg.foot_location.row3}''')

        self.get_logger().info(f'command_callback, horizontal_velocity: {msg.horizontal_velocity}')
        self.get_logger().info(f'command_callback, robot_speed: {msg.robot_speed}')
        self.get_logger().info(f'command_callback, attitude: {msg.attitude}')

        self.current_command = msg

    def imu_callback(self, msg):
        self.quat_orientation = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

    def control_loop_callback(self):
        """
        Main control loop executed periodically.
        """
        if self.current_command is None:
            return

        # Update the operating state based on the command
        if self.current_command.activate_event:
            self.state.behavior_state = self.activate_transition_mapping[self.state.behavior_state]
        elif self.current_command.trot_event:
            self.state.behavior_state = self.trot_transition_mapping[self.state.behavior_state]
        elif self.current_command.hop_event:
            self.state.behavior_state = self.hop_transition_mapping[self.state.behavior_state]

        self.get_logger().info(f"Behavior state: {self.state.behavior_state}")

        # Manage dance state
        self.dance_active(self.current_command)
        self.pseudo_dance_active(self.current_command)

        # Perform actions based on the behavior state
        if self.state.behavior_state == BehaviorState.TROT:
            self.handle_trot_state()
        elif self.state.behavior_state == BehaviorState.HOP:
            self.handle_hop_state()
        elif self.state.behavior_state == BehaviorState.FINISHHOP:
            self.handle_finishhop_state()
        elif self.state.behavior_state == BehaviorState.REST:
            self.handle_rest_state()

        # Increment ticks
        self.state.ticks += 1

        self.publish_joints_command()

    def handle_trot_state(self):
        """
        Handle the TROT behavior state.
        """
        self.state.foot_locations, contact_modes = self.step_gait(
            self.state,
            self.current_command,
        )

        rotated_foot_locations = (
            euler2mat(
                self.current_command.roll,
                self.current_command.pitch,
                0.0
            ) @ self.state.foot_locations
        )

        # Apply tilt compensation
        rotated_foot_locations = self.apply_tilt_compensation(rotated_foot_locations)

        # Update joint angles
        self.state.joint_angles = self.inverse_kinematics(
            rotated_foot_locations, self.config
        )

    def handle_hop_state(self):
        """
        Handle the HOP behavior state.
        """
        self.state.foot_locations = (
            self.config.default_stance
            + np.array([0, 0, -0.03])[:, np.newaxis]
        )
        self.state.joint_angles = self.inverse_kinematics(
            self.state.foot_locations, self.config
        )

    def handle_finishhop_state(self):
        """
        Handle the FINISHHOP behavior state.
        """
        self.state.foot_locations = (
            self.config.default_stance
            + np.array([0, 0, -0.105])[:, np.newaxis]
        )
        self.state.joint_angles = self.inverse_kinematics(
            self.state.foot_locations, self.config
        )

    def handle_rest_state(self):
        """
        Handle the REST behavior state.
        """
        yaw_proportion = self.current_command.yaw_rate / self.config.max_yaw_rate
        self.smoothed_yaw += (
            self.config.dt
            * clipped_first_order_filter(
                self.smoothed_yaw,
                yaw_proportion * -self.config.max_stance_yaw,
                self.config.max_stance_yaw_rate,
                self.config.yaw_time_constant,
            )
        )

        if not self.dance_active_state:
            self.state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, self.current_command.height])[:, np.newaxis]
            )
            rotated_foot_locations = (
                euler2mat(
                    self.current_command.roll,
                    self.current_command.pitch,
                    self.smoothed_yaw,
                ) @ self.state.foot_locations
            )
        else:
            foot_location = self.current_command.foot_location
            location_buf = np.array([foot_location.row1, foot_location.row2, foot_location.row3])
            if (abs(self.current_command.robot_speed[0])<0.01) and (abs(self.current_command.robot_speed[1])<0.01):
                self.state.foot_locations = location_buf
            else:
                self.current_command.horizontal_velocity[0] = self.current_command.robot_speed[0]
                self.current_command.horizontal_velocity[1] = self.current_command.robot_speed[1]
                self.state.foot_locations, contact_modes = self.step_gait(self.state, self.current_command)
            
            rotated_foot_locations = (
                euler2mat(
                    self.current_command.attitude[0],
                    self.current_command.attitude[1],
                    self.current_command.attitude[2],
                ) @ self.state.foot_locations
            )

        # Apply tilt compensation
        rotated_foot_locations = self.apply_tilt_compensation(rotated_foot_locations)

        # Update joint angles
        self.state.joint_angles = self.inverse_kinematics(
            rotated_foot_locations, self.config
        )

    def apply_tilt_compensation(self, foot_locations):
        """
        Apply tilt compensation to the foot locations.
        """
        roll, pitch, yaw = quat2euler(self.quat_orientation)
        correction_factor = 0.8
        max_tilt = 0.4
        roll_compensation = correction_factor * np.clip(-roll, -max_tilt, max_tilt)
        pitch_compensation = correction_factor * np.clip(-pitch, -max_tilt, max_tilt)
        rmat = euler2mat(roll_compensation, pitch_compensation, 0)
        return rmat.T @ foot_locations

    def step_gait(self, state, command):
        """Calculate the desired foot locations for the next timestep

        Returns
        -------
        Numpy array (3, 4)
            stanford of new foot locations.
        """
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            foot_location = state.foot_locations[:, leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks) / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    state,
                    command
                )
            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations, contact_modes
   

    def dance_active(self, command):
        """
        Manage dance activation state.
        """
        if command.dance_activate_event:
            self.dance_active_state = not self.dance_active_state

    def pseudo_dance_active(self, command):
        """
        Manage pseudo-dance activation state.
        """
        if command.pseudo_dance_event:
            self.dance_active_state = True

    def publish_joints_command(self):
        joints_cmd_msg = JointTrajectory()
        joints_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        joints_cmd_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.state.joint_angles.flatten().tolist()
        point.time_from_start = rclpy.duration.Duration(seconds=1.0 / 60.0).to_msg()

        joints_cmd_msg.points.append(point)
        self.joint_commands_publisher.publish(joints_cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    config = Configuration()
    node = StanfordController(config, four_legs_inverse_kinematics)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
