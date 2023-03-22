import os
import sys
import numpy as np

# Stanford Quadruped
import ament_index_python.packages as packages
package_path = packages.get_package_share_directory('mini_pupper_driver')
sys.path.append(package_path+"/mini_pupper_driver")
sys.path.extend([os.path.join(root, name) for root, dirs, _ in os.walk(package_path+"/mini_pupper_driver") for name in dirs])
from src.Controller import Controller
from src.State import State
from src.Command import Command
from src.MovementScheme import MovementScheme
from pupper.MovementGroup import MovementLib
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics

# ROS 2 related packages
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


class VelocityToServoController(Node):

    def __init__(self):
        super().__init__("velocity_to_servo_controller")

        # Parameters
        self.receive_topic_name = "/cmd_vel"
        self.FIRST_TIME_FLAG = 1
        self.TIMER_INTERVAL = 0.02

        # Motion Control
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_linear_z = 0.0
        self.target_angular_x = 0.0
        self.target_angular_y = 0.0
        self.target_angular_z = 0.0
        self.quat_orientation = np.array([1, 0, 0, 0])

        # Standford Quadruped Initialization
        self.config = Configuration()
        self.hardware_interface = HardwareInterface()
        self.movement_ctl = MovementScheme(MovementLib)
        self.controller = Controller(self.config, four_legs_inverse_kinematics)
        self.state = State()
        self.initial_angles = self.state.joint_angles

        # Publisher & Subscriber
        
        self.publisher = self.create_publisher(JointState, 'joint_command', 50)
        self.subscriber = self.create_subscription(Twist, self.receive_topic_name, self.listener_callback, 1)
        self.subscriber  # prevent unused variable warning

        # Timer
        self.timer = self.create_timer(
            self.TIMER_INTERVAL, self.data_processor)

        # Human Behaviors Counter
        self.counter = 0

    def publish_joint_positions(self, joint_angles):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.position = joint_angles.flatten().tolist()
        self.publisher.publish(joint_state_msg)

    def listener_callback(self, twist_msg):
        self.target_linear_x = twist_msg.linear.x
        self.target_linear_y = twist_msg.linear.y
        self.target_angular_z = twist_msg.angular.z
        self.target_linear_z = twist_msg.linear.z
        self.target_angular_x = twist_msg.angular.x
        self.target_angular_y = twist_msg.angular.y

    def data_processor(self):
        # IMU TODO
        self.state.quat_orientation = self.quat_orientation

        # Command Formation
        command = Command()
        command.horizontal_velocity = np.array(
            [self.target_linear_x, self.target_linear_y])
        command.yaw_rate = self.target_angular_z

        # Simulate Human Behaviors
        if (self.counter == 0):
            self.counter += 1
        elif (self.counter == 1):
            command.activate_event = True
            self.counter += 1
        elif (self.counter == 2):
            self.initial_angles = self.state.joint_angles
            self.counter += 1
        elif (self.counter == 3):
            command.trot_event = True
            self.counter += 1
        else:
            pass

        # If Speed is 0, Stop Trotting.
        STOP = False
        if (self.target_linear_x == 0 and self.target_linear_y == 0 and self.target_angular_z == 0):
            STOP = True

        # Main
        if (command.activate_event is not True):
            # Movement Scheme
            movement_switch = command.dance_switch_event
            # May be used in the future TODO
            # gait_state = command.trot_event
            # dance_state = command.dance_activate_event
            # shutdown_signal = command.shutdown_signal
            self.movement_ctl.runMovementScheme(movement_switch)
            foot_location = self.movement_ctl.getMovemenLegsLocation()
            attitude_location = self.movement_ctl.getMovemenAttitude()
            robot_speed = self.movement_ctl.getMovemenSpeed()
            self.controller.run(self.state, command,
                                foot_location, attitude_location, robot_speed)

            # Hardware Control
            if not STOP:
                self.publish_joint_positions(self.state.joint_angles)
            else:
                self.publish_joint_positions(self.initial_angles)

    def print_info_gait(self):
        print("Summary of gait parameters:")
        print("overlap time: ", self.config.overlap_time)
        print("swing time: ", self.config.swing_time)
        print("z clearance: ", self.config.z_clearance)
        print("x shift: ", self.config.x_shift)


def cmd_dump(cmd):
    """
       print cmd format message
       Parameter: cmd
       return : None
    """
    print("\nGet PS4 command :")
    print("horizontal_velocity: ", cmd.horizontal_velocity)
    print("yaw_rate ", cmd.yaw_rate)
    print("height", cmd.height)
    print("pitch ", cmd.pitch)
    print("roll ", cmd.roll)
    print("activation ", cmd.activation)
    print("hop_event ", cmd.hop_event)
    print("trot_event ", cmd.trot_event)
    print("activate_event ", cmd.activate_event)


def main(args=None):
    rclpy.init(args=args)
    velocity_to_servo_controller = VelocityToServoController()
    rclpy.spin(velocity_to_servo_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
