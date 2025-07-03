#!/usr/bin/env python3

import rclpy
import time as pytime
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from mini_pupper_interfaces.msg import Tracking, TrackingArray, Command, Matrix3x4
from tf_transformations import euler_from_quaternion
import math
import numpy as np
import subprocess
from enum import Enum
from stanford_controller.Config import Configuration


class PID:
    def __init__(self, Kp, Ki, Kd):
        self.K = [Kp, Ki, Kd]
        self.prev_error = 0
        self.integral = 0
        # Add storage for component values
        self.last_p = 0
        self.last_i = 0
        self.last_d = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        # Store individual components
        self.last_p = self.K[0] * error
        self.last_i = self.K[1] * self.integral
        self.last_d = self.K[2] * derivative
        
        self.prev_error = error
        return self.last_p + self.last_i + self.last_d
    

class MovementNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_movement_node')
        self.get_logger().info("Movement Node Created")

        self.config = Configuration()
        # used to detect rising/falling edges on “non-zero” cmd_vel
        self.prev_zero = True
        self.cmdpub = self.create_publisher(Command, '/robot_command', 10)

        self.tracksub = self.create_subscription(TrackingArray, "/tracking_array", self.tracking_callback, 10)
        self.imusub = self.create_subscription(Imu, "/imu/qdata", self.imu_callback, 10)
        
        # Tracking variables
        self.detected = False
        self.center_x = 0.0
        self.center_y = 0.0
        self.bounding_area = 0.0

        self.turn_pid = PID(5.0, 0.0, 0.1)
        # Average derivative is around 5.0 (0-10)
        self.turn_timer = self.create_timer(0.015, self.turn_callback)
        self.last_turn = 0.0

        self.pid_log_timer = self.create_timer(0.2, self.log_pid_data)

        self.current_yaw = 0.0
        self.fov_deg = 62.2
        self.fov_rad = math.radians(self.fov_deg)
        self.last_target_yaw = None
        self.turn_decay = 0.5
        self.turn_clamp = 2.0
        self.turn_stable_minimum = 0.5
        self.dead = False
        self.last_turn_time = self.get_clock().now()
    
    def imu_callback(self, msg: Imu):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw

    def log_pid_data(self):
        if self.detected:
            if not self.dead:
                self.get_logger().info(
                    f"P={self.turn_pid.last_p:.3f} "
                    f"I={self.turn_pid.last_i:.3f} "
                    f"D={self.turn_pid.last_d:.3f} "
                    f"Total={self.turn_pid.last_p + self.turn_pid.last_i + self.turn_pid.last_d:.3f} "
                    f"Yaw={self.current_yaw:.3f}"
                    f"\n"
                )
            else:
                self.get_logger().info("DEAD")


    def tracking_callback(self, msg: TrackingArray):
        if not msg.tracks:
            self.detected = False
            return

        # Pick detection with highest confidence
        choice = max(msg.tracks, key=lambda t: t.bounding_area)
        
        self.center_x = choice.center_x
        self.center_y = choice.center_y
        self.bounding_area = choice.bounding_area
        self.detected = True
    

    def turn_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_turn_time).nanoseconds / 1e9
        self.last_turn_time = now

        angular = 0.0
        yaw_error = 0.0
        output_raw = 0.0

        if self.detected:
            offset_angle = (self.center_x - 0.5) * self.fov_rad
            desired_yaw = self.current_yaw + offset_angle
            self.last_target_yaw = desired_yaw

            yaw_error = math.atan2(
                math.sin(desired_yaw - self.current_yaw),
                math.cos(desired_yaw - self.current_yaw)
            )

        elif self.last_target_yaw is not None:
            yaw_error = math.atan2(
                math.sin(self.last_target_yaw - self.current_yaw),
                math.cos(self.last_target_yaw - self.current_yaw)
            )

        else:
            # No detection and no remembered target: decay old command
            self.dead = True
            self.last_turn *= self.turn_decay
            angular = self.last_turn
            cmd = self.create_command(ang=angular)
            self.cmdpub.publish(cmd)
            return

        # Run PID even if error is small
        output_raw = self.turn_pid.compute(-yaw_error, dt)

        # Apply deadband: if the output is too small, suppress it
        if abs(output_raw) < self.turn_stable_minimum:
            angular = 0.0
            self.dead = True
        else:
            angular = np.clip(output_raw, -self.turn_clamp, self.turn_clamp)
            self.dead = False

        self.last_turn = angular
        cmd = self.create_command(ang=angular)
        self.cmdpub.publish(cmd)

    def create_command(self, vel=[0.0,0.0], ang=0.0, pitch=0.0):
        cmd = Command()
        cmd.height = self.config.default_z_ref

        # Default standing locations

        default_stance = self.config.default_stance
        cmd.legs_location = Matrix3x4(
            row1=default_stance[0].tolist(),
            row2=default_stance[1].tolist(),
            row3=default_stance[2].tolist()
        )

        # Clamp velocity and angular velocity
        x_vel = float(np.clip(
            vel[0],
            -self.config.max_x_velocity,
            self.config.max_x_velocity,
        ))
        y_vel = float(np.clip(
            vel[1],
            -self.config.max_y_velocity,
            self.config.max_y_velocity,
        ))
        yaw_rate = float(np.clip(
            ang,
            -self.config.max_yaw_rate,
            self.config.max_yaw_rate,
        ))
        cmd.horizontal_velocity = np.array([x_vel, y_vel])
        cmd.yaw_rate = yaw_rate
        cmd.roll = 0.0
        cmd.pitch = pitch
        cmd.yaw = 0.0 # cmd.yaw is only used when the robot is stationary so not used for tracking

        # IMPORTANT
        # Detect zero↔non-zero edge and fire trot_event only once
        is_zero = self._vel_zero(vel, yaw_rate)
        cmd.trot_event = (self.prev_zero != is_zero)
        self.prev_zero = is_zero
        return cmd

    def _vel_zero(self, vel, ang):
        """
        Returns True if both horizontal velocity and yaw_rate are approximately zero.
        """
        return np.allclose(
            [vel[0], vel[1], ang],
            0.0,
            atol=1e-3
        )


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Graceful stop using Command message
        stop_cmd = node.create_command(vel=[0.0, 0.0], ang=0.0)
        node.cmdpub.publish(stop_cmd)
        node.get_logger().info("Stop command sent to robot_command")
        pytime.sleep(0.5)  # Give it time to send
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()