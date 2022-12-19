import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from MangDang.mini_pupper.HardwareInterface import HardwareInterface

class SERVO_INTERFACE(Node):
    def __init__(self):
        super().__init__('servo_interface')
        self.subscriber = self.create_subscription(JointTrajectory, '/joint_group_effort_controller/joint_trajectory', self.cmd_callback, 10)
        self.hardware_interface = HardwareInterface()
        
    def cmd_callback(self, msg):
        joint_positions = msg.points[0].positions
        lf1_position = joint_positions[0]
        lf2_position = joint_positions[1]
        lf3_position = joint_positions[2]
        rf1_position = joint_positions[3]
        rf2_position = joint_positions[4]
        rf3_position = joint_positions[5]
        lb1_position = joint_positions[6]
        lb2_position = joint_positions[7]
        lb3_position = joint_positions[8]
        rb1_position = joint_positions[9]
        rb2_position = joint_positions[10]
        rb3_position = joint_positions[11]
        
        joint_angles = np.array([
        [rf1_position, lf1_position, rb1_position, lb1_position],
        [rf2_position, lf2_position, rb2_position, lb2_position],
        [rf2_position + rf3_position, lf2_position + lf3_position,
         rb2_position + rb3_position, lb2_position + lb3_position]
        ])
        self.hardware_interface.set_actuator_postions(joint_angles)

def main(args=None):
    rclpy.init(args = args)
    servo_interface_node = SERVO_INTERFACE()
    rclpy.spin(servo_interface_node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()