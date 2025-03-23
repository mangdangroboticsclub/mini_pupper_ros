import rclpy
from rclpy.node import Node
from .gait_controller import GaitController
from .stance_controller import StanceController
from .swing_controller import SwingController

from .Kinematics import four_legs_inverse_kinematics
from .Utilities import clipped_first_order_filter
from .Utilities import convert_to_JTP_positions
from .State import BehaviorState, State

from MangDang.mini_pupper.Config import Configuration

import numpy as np
from transforms3d.euler import euler2mat, quat2euler

from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from mini_pupper_interfaces.msg import Command
from mini_pupper_interfaces.msg import Matrix3x4

class StanfordControllerNode(Node):
    """
    ROS 2 Node for StanfordController
    """

    def __init__(self, config, inverse_kinematics):
        super().__init__('stanford_controller_node')

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

        self.state = State()  
        self.quat_orientation = np.array([1, 0, 0, 0])

    def imu_callback(self, msg):
        self.quat_orientation = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

    def dance_active(self, command):
        if command.dance_activate_event == True:
            if self.dance_active_state == False:
                self.dance_active_state = True
            else:
                self.dance_active_state = False
        return True

    def pseudo_dance_active(self, command):
        if command.pseudo_dance_event == True:
            self.dance_active_state = True

    def step_gait(self, state, command):
        """Calculate the desired foot locations for the next timestep

        Returns
        -------
        Numpy array (3, 4)
            Matrix of new foot locations.
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

    def command_callback(self, command):
        """Steps the controller forward one timestep

        Parameters
        ----------
        controller : Controller
            Robot controller object.
        """

        ########## Update operating state based on command ######
        if command.activate_event:
            self.state.behavior_state = self.activate_transition_mapping[self.state.behavior_state]
        elif command.trot_event:
            self.state.behavior_state = self.trot_transition_mapping[self.state.behavior_state]
        elif command.hop_event:
            self.state.behavior_state = self.hop_transition_mapping[self.state.behavior_state]

        #disp.show_state(state.behavior_state)
        self.dance_active(command)
        self.pseudo_dance_active(command)

        if self.state.behavior_state == BehaviorState.TROT:
            self.state.foot_locations, contact_modes = self.step_gait(
                self.state,
                command,
            )

            # Apply the desired body rotation
            rotated_foot_locations = (
                euler2mat(
                    command.roll, command.pitch, 0.0
                )
                @ self.state.foot_locations
            )

            # Construct foot rotation matrix to compensate for body tilt
            (roll, pitch, yaw) = quat2euler(self.state.quat_orientation)
            correction_factor = 0.8
            max_tilt = 0.4
            roll_compensation = correction_factor * np.clip(-roll, -max_tilt, max_tilt)
            pitch_compensation = correction_factor * np.clip(-pitch, -max_tilt, max_tilt)
            rmat = euler2mat(roll_compensation, pitch_compensation, 0)

            rotated_foot_locations = rmat.T @ rotated_foot_locations

            self.state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )

        elif self.state.behavior_state == BehaviorState.HOP:
            self.state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, -0.03])[:, np.newaxis]
            )
            self.state.joint_angles = self.inverse_kinematics(
                self.state.foot_locations, self.config
            )

        elif self.state.behavior_state == BehaviorState.FINISHHOP:
            self.state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, -0.105])[:, np.newaxis]
            )
            self.state.joint_angles = self.inverse_kinematics(
                self.state.foot_locations, self.config
            )

        elif self.state.behavior_state == BehaviorState.REST:
            yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            self.smoothed_yaw += (
                self.config.dt
                * clipped_first_order_filter(
                    self.smoothed_yaw,
                    yaw_proportion * -self.config.max_stance_yaw,
                    self.config.max_stance_yaw_rate,
                    self.config.yaw_time_constant,
                )
            )

            if self.dance_active_state == False:
                #  Set the foot locations to the default stance plus the standard height
                self.state.foot_locations = (
                    self.config.default_stance
                    + np.array([0, 0, command.height])[:, np.newaxis]
                 )
                # Apply the desired body rotation
                rotated_foot_locations = (
                    euler2mat(
                        command.roll,
                        command.pitch,
                        self.smoothed_yaw,
                    )
                    @ self.state.foot_locations
                )
            else:
                location_buf = self.get_2d_foot_locations(command)
                if (abs(command.robot_speed[0])<0.01) and (abs(command.robot_speed[1])<0.01):
                    self.state.foot_locations = location_buf
                else:
                    command.horizontal_velocity[0] = command.robot_speed[0]
                    command.horizontal_velocity[1] = command.robot_speed[1]
                    self.state.foot_locations,contact_modes = self.step_gait(self.state, command)

                rotated_foot_locations = (
                    euler2mat(
                        command.attitude[0],
                        command.attitude[1],
                        command.attitude[2],
                    )
                    @ self.state.foot_locations
                )

            # Construct foot rotation matrix to compensate for body tilt
            (roll, pitch, yaw) = quat2euler(self.state.quat_orientation)
            correction_factor = 0.8
            max_tilt = 0.4
            roll_compensation = correction_factor * np.clip(-roll, -max_tilt, max_tilt)
            pitch_compensation = correction_factor * np.clip(-pitch, -max_tilt, max_tilt)
            rmat = euler2mat(roll_compensation, pitch_compensation, 0)

            rotated_foot_locations = rmat.T @ rotated_foot_locations

            self.state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )

        self.state.ticks += 1
        self.state.pitch = command.pitch
        self.state.roll = command.roll
        self.state.height = command.height
        self.publish_joints_command()

    def get_2d_foot_locations(self, command):
        location = command.foot_location
        return np.array([location.row1, location.row2, location.row3])

    def publish_joints_command(self):
        joints_cmd_msg = JointTrajectory()
        joints_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        joints_cmd_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = convert_to_JTP_positions(self.state.joint_angles)
        point.time_from_start = rclpy.duration.Duration(seconds=1.0 / 60.0).to_msg()

        joints_cmd_msg.points.append(point)
        self.joint_commands_publisher.publish(joints_cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    config = Configuration()
    node = StanfordControllerNode(config, four_legs_inverse_kinematics)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
