#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory 
from sensor_msgs.msg import JointState

class TrajectoryToState(Node):
    def __init__(self):
        super().__init__('stanford_joint_trajectory_to_states')
        self.subscription = self.create_subscription(JointTrajectory, '/joint_group_effort_controller/joint_trajectory', self.joint_trajectory_callback, 10)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

    def joint_trajectory_callback(self, msg):
        if not msg.points or not msg.points[0].positions:
            self.get_logger().warn('Received empty JointTrajectory message, skipping publication.')
            return

        joint_state = JointState()
        joint_state.header = msg.header
        joint_state.name = msg.joint_names
        joint_state.position = msg.points[0].positions
        joint_state.velocity = msg.points[0].velocities
        joint_state.effort = msg.points[0].effort

        self.publisher.publish(joint_state)

def main():
    rclpy.init()
    node = TrajectoryToState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
