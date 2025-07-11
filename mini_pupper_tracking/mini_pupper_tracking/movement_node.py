#!/usr/bin/env python3

import rclpy
import time as pytime
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mini_pupper_interfaces.msg import Tracking, TrackingArray, Command, Matrix3x4
from tf_transformations import euler_from_quaternion
import math
import numpy as np
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

        # Parameter Fetching
        self.declare_parameter('yaw.Kp', 5.0)
        self.declare_parameter('yaw.Kd', 0.1)
        self.declare_parameter('yaw.decay', 0.5)
        self.declare_parameter('yaw.clamp', 2.0)
        self.declare_parameter('yaw.stable_minimum', 0.5)
        self.declare_parameter('yaw.tracking_enabled', False)
        self.declare_parameter('pitch.alpha', 0.1)
        self.declare_parameter('pitch.gain', 1.0)
        self.declare_parameter('pitch.decay', 0.5)
        self.declare_parameter('pitch.camera_deadband', 0.020)
        self.declare_parameter('pitch.tracking_enabled', False)

        # Configuration
        self.config = Configuration()
        self.prev_zero = True  # Used to detect zero↔non-zero transitions
        self.cmdpub = self.create_publisher(Command, '/robot_command', 10)

        #  Subscriptions
        self.tracksub = self.create_subscription(TrackingArray, "/tracking_array", self.tracking_callback, 10)
        self.imusub = self.create_subscription(Imu, "/imu/qdata", self.imu_callback, 10)

        # Detection
        self.detected = False
        self.center_x = 0.0
        self.top_y = 0.0
        self.bounding_area = 0.0

        # Camera FOV
        self.fov_deg = 62.2
        self.fov_rad = math.radians(self.fov_deg)
        self.vertical_fov_deg = 48.8
        self.vertical_fov_rad = math.radians(self.vertical_fov_deg)

        # IMU Orientation
        self.current_yaw = 0.0

        # Yaw Control
        self.yaw_tracking_enabled = self.get_parameter('yaw.tracking_enabled').value
        self.yaw_pid = PID(self.get_parameter('yaw.Kp').value, 0.0, self.get_parameter('yaw.Kd').value)
        self.last_yaw_rate = 0.0
        self.last_target_yaw = None
        self.yaw_decay = self.get_parameter('yaw.decay').value
        self.yaw_clamp = self.get_parameter('yaw.clamp').value
        self.yaw_stable_minimum = self.get_parameter('yaw.stable_minimum').value # <-- increase
        self.last_turn_time = self.get_clock().now()
        self.yaw_dead = False

        # Pitch Control (Smooth Version)
        self.pitch_tracking_enabled = self.get_parameter('pitch.tracking_enabled').value
        self.pitch_value = 0.0
        self.last_pitch_time = self.get_clock().now()
        self.pitch_dead = False
        self.current_pitch = 0.0        
        self.smoothed_offset = 0.0
        self.wanted_top_y = 0.4

        # Pitch parameters
        self.pitch_alpha = self.get_parameter('pitch.alpha').value # Smoothing factor (0.1=very smooth, 0.5=responsive)
        self.pitch_gain = self.get_parameter('pitch.gain').value # Proportional gain (0.2=gentle, 0.8=aggressive)
        self.pitch_decay = self.get_parameter('pitch.decay').value # To gradually smooth if person lost (0.5=very aggressive, 0.9=smooth)
        self.pitch_camera_deadband = self.get_parameter('pitch.camera_deadband').value # <-- increase

        self.max_pitch_delta = 10.0 # Currently unused

        # Timers
        self.yaw_timer = self.create_timer(0.015, self.yaw_callback)
        self.pitch_timer = self.create_timer(0.015, self.pitch_callback)
        self.command_timer = self.create_timer(0.015, self.command_callback)
        self.log_timer = self.create_timer(1.0, self.log_data)
    
    def command_callback(self):
        yaw_rate = self.last_yaw_rate if self.yaw_tracking_enabled else 0.0
        pitch = self.pitch_value if self.pitch_tracking_enabled else 0.0
        
        cmd = self.create_command(yaw_rate=yaw_rate, pitch=pitch)
        self.cmdpub.publish(cmd)

    def pitch_callback(self):
        if self.detected:
            # Calculate offset angle
            offset_angle = (self.wanted_top_y - self.top_y) * self.vertical_fov_rad

            # Exponential smoothing
            self.smoothed_offset = (
                self.pitch_alpha * offset_angle +
                (1 - self.pitch_alpha) * self.smoothed_offset
            )

            # Apply control only if outside deadband
            if abs(self.smoothed_offset) > self.pitch_camera_deadband:
                target_pitch = self.current_pitch + self.pitch_gain * self.smoothed_offset
                self.raw_pitch_delta = target_pitch - self.pitch_value
                delta = np.clip(
                    target_pitch - self.pitch_value,
                    -self.max_pitch_delta,
                    self.max_pitch_delta
                )
                self.pitch_value += delta
                self.pitch_dead = False
            else:
                self.pitch_dead = True
        else:
            self.smoothed_offset *= self.pitch_decay
            self.pitch_dead = True

        self.pitch_value = float(np.clip(
            self.pitch_value,
            -np.pi/10,
            np.pi/10,
        ))


    def yaw_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_turn_time).nanoseconds / 1e9
        self.last_turn_time = now

        yaw_error = 0.0
        if self.detected:
            offset_angle = (0.5 - self.center_x) * self.fov_rad
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
            self.yaw_dead = True
            self.last_yaw_rate *= self.yaw_decay
            return

        output_raw = self.yaw_pid.compute(yaw_error, dt)

        if abs(output_raw) < self.yaw_stable_minimum:
            self.last_yaw_rate = 0.0
            self.yaw_dead = True
        else:
            self.last_yaw_rate = float(np.clip(output_raw, -self.yaw_clamp, self.yaw_clamp))
            self.yaw_dead = False

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw # Store current yaw for tracking
        self.current_pitch = pitch # Store current pitch for tracking

    def log_data(self):
        if self.detected:
            self.get_logger().info(
                f"Center X: {self.center_x:.2f}, "
                f"Top Y: {self.top_y:.2f}, "
                f"Bounding Area: {self.bounding_area:.2f}, "
                f"Pitch Value: {self.pitch_value:.2f}, "
                f"Yaw Rate: {self.last_yaw_rate:.2f}"
            )


    def tracking_callback(self, msg: TrackingArray):
        if not msg.tracks:
            self.detected = False
            return

        # Pick detection with highest confidence
        choice = max(msg.tracks, key=lambda t: t.bounding_area)
        
        self.center_x = choice.center_x
        self.top_y = choice.top_y
        self.bounding_area = choice.bounding_area
        self.detected = True

    def create_command(self, vel=[0.0,0.0], yaw_rate=0.0, pitch=0.0):
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
        yaw_rate_clipped = float(np.clip(
            yaw_rate,
            -self.config.max_yaw_rate,
            self.config.max_yaw_rate,
        ))
        pitch_clipped = float(np.clip(
            pitch,
            -np.pi/10,
            np.pi/10,
        ))
        cmd.horizontal_velocity = np.array([x_vel, y_vel])
        cmd.yaw_rate = yaw_rate_clipped
        cmd.roll = 0.0
        cmd.pitch = pitch_clipped
        cmd.yaw = 0.0 # cmd.yaw is only used when the robot is stationary so not used for tracking

        # IMPORTANT
        # Detect zero↔non-zero edge and fire trot_event only once
        is_zero = self._vel_zero(vel, yaw_rate)
        cmd.trot_event = (self.prev_zero != is_zero)
        self.prev_zero = is_zero
        return cmd

    def _vel_zero(self, vel, yaw_rate):
        """
        Returns True if both horizontal velocity and yaw_rate are approximately zero.
        """
        return np.allclose(
            [vel[0], vel[1], yaw_rate],
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
        stop_cmd = node.create_command(vel=[0.0, 0.0], yaw_rate=0.0)
        node.cmdpub.publish(stop_cmd)
        node.get_logger().info("Stop command sent to robot_command")
        pytime.sleep(0.5)  # Give it time to send
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()