#!/usr/bin/python3

from __future__ import annotations
from multiprocessing import Event, Lock
import time

from rospy import Subscriber, Publisher, Timer, Duration, init_node, get_param, on_shutdown, spin
from mini_pupper import MiniPupper, Mode
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String, Bool


class ServoControllerV2:

    def __init__(self):
        init_node('servo_controller', anonymous=True)
        self.minipupper: MiniPupper = MiniPupper.parse_obj(get_param("legs")).activate()
        self.should_stand = get_param("~should_stand")
        self.is_standing = self.should_stand
        self.walker: Subscriber = self.register_walker() if self.should_stand else None
        self.poser: Subscriber = self.register_poser()
        self.pose_reporter = Publisher('/pose/is_standing', Bool, queue_size=1)
        self.pose_timer = Timer(Duration(1), self.report_pose)
        self.shutdown_pause: int = 1
        self.movement_lock: Lock = Lock()
        self.on_movement: Event = Event()
        on_shutdown(self.release_servos)

    @property
    def pose_state_message(self):
        msg = Bool()
        msg.data = self.is_standing
        return msg

    def release_servos(self):
        self.unregister_walker()
        self.unregister_poser()
        self.minipupper.sit()
        # Give a time for servos to change the pose
        time.sleep(self.shutdown_pause)
        # Release servos
        self.minipupper.deactivate()

    def handle_pose(self, payload) -> None:
        if payload.data == Mode.CALIBRATE:
            self.unregister_walker()
        elif payload.data == Mode.SIT:
            self.unregister_walker()
            self.minipupper.sit()
            self.is_standing = False
        elif payload.data == Mode.STAND and not self.walker:
            self.walker = self.register_walker()
            self.is_standing = True

    def handle_movement(self, payload) -> None:
        points = payload.points[0] if payload.points else []
        # Single DOF joint positions for each joint relative to their "0" position.
        # The units depend on the specific joint type: radians for revolute or
        # continuous joints, and meters for prismatic joints.
        joint_positions = points.positions if points.positions else []
        if not self.on_movement.is_set():
            self.unregister_walker()
            self.minipupper.move(joint_positions, self.on_movement)
            self.walker = self.register_walker()
        else:
            self.minipupper.move(joint_positions, self.on_movement)

    def register_poser(self) -> Subscriber:
        return Subscriber(
            "/pose/change",
            String,
            self.handle_pose,
            queue_size=1
        )

    def unregister_poser(self) -> ServoControllerV2:
        if self.poser:
            self.poser.unregister()
            self.poser = None
        return self

    def register_walker(self) -> Subscriber:
        return Subscriber(
            "/joint_group_position_controller/command",
            JointTrajectory,
            self.handle_movement,
            queue_size=1
        )

    def unregister_walker(self) -> ServoControllerV2:
        if self.walker:
            self.walker.unregister()
            self.walker = None
        return self

    def report_pose(self, event=None):
        self.pose_reporter.publish(self.pose_state_message)

    def run(self) -> None:
        spin()


if __name__ == '__main__':
    ServoControllerV2().run()
