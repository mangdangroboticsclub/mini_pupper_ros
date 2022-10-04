from __future__ import print_function

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose
from rospy import get_param, Subscriber, \
    Publisher, Timer, Duration, init_node, \
    is_shutdown, logerr, loginfo, sleep, spin

from threading import Condition, Thread
from tf_conversions import transformations


msg = """
Reading a key from the /key subscriber and Publishing to Twist and Pose!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

Body pose:
---------------------------
1/2 : move the body forward/back (+/-x)
3/4 : move the body right/left (+/-y)

5/6 : move the body up/down (+/-z)

a/s : body's roll
d/f : body's pitch
g/h : body's yaw

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

r/v : increase/decrease body's pose translation by 10%
t/b : increase/decrease body's pose angular speed by 10%
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

poseBindings = {
    '1': (1, 0, 0, 0, 0, 0),
    '2': (-1, 0, 0, 0, 0, 0),
    '3': (0, 1, 0, 0, 0, 0),
    '4': (0, -1, 0, 0, 0, 0),
    '5': (0, 0, 1, 0, 0, 0),
    '6': (0, 0, -1, 0, 0, 0),
    'a': (0, 0, 0, 1, 0, 0),
    's': (0, 0, 0, -1, 0, 0),
    'd': (0, 0, 0, 0, 1, 0),
    'f': (0, 0, 0, 0, -1, 0),
    'g': (0, 0, 0, 0, 0, 1),
    'h': (0, 0, 0, 0, 0, -1),
}

speedPoseBindings = {
    'r': (1.1, 1),
    'v': (.9, 1),
    't': (1, 1.1),
    'b': (1, .9),
}


class RemoteTeleopThread(Thread):
    def __init__(self):
        super(RemoteTeleopThread, self).__init__()

        init_node("remote_teleop")
        robot_name = get_param("~/robot_name", "/")
        twist_publisher_name = get_param("~/twist_publisher_name", robot_name + "cmd_vel")
        pose_publisher_name = get_param("~/pose_publisher_name", robot_name + "body_pose")
        teleop_publisher_name = get_param("~/teleop_publisher_name", robot_name + "teleop_status")
        key_subscriber_name = get_param("~/key_subscriber_name", robot_name + "key")

        self.key_subscriber = Subscriber(
            key_subscriber_name,
            String,
            callback=self.on_key,
            queue_size=1
        )
        self.teleop_state_publisher = Publisher(teleop_publisher_name, Bool, queue_size=1)
        self.twist_publisher = Publisher(twist_publisher_name, Twist, queue_size=1)
        self.pose_publisher = Publisher(pose_publisher_name, Pose, queue_size=1)
        self.teleop_state_timer = Timer(Duration(1), self.publish_teleop_state)

        self.speed = get_param("~/speed", 0.5)
        self.turn = get_param("~/turn", 1.0)
        self.pose_speed = get_param("~/pose_speed", 0.01)
        self.pose_turn = get_param("~/pose_turn", 0.1)
        self.rate = get_param("~/repeat_rate", 0.0)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.pose_roll = 0.0
        self.pose_pitch = 0.0
        self.pose_yaw = 0.0
        self.condition = Condition()
        self.is_teleop_ready = False
        self.done = False
        self.delay_wait_print = 4

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        self.timeout = 1.0 / self.rate if self.rate != 0.0 else None

        self.start()

    @property
    def teleop_ready_message(self):
        msg = Bool()
        msg.data = self.is_teleop_ready
        return msg

    def publish_teleop_state(self, event=None):
        self.teleop_state_publisher.publish(self.teleop_ready_message)

    @property
    def twist_connections(self):
        return self.twist_publisher.get_num_connections()

    @property
    def pose_connections(self):
        return self.pose_publisher.get_num_connections()

    @property
    def pose(self):
        pose = Pose()

        pose.position.x = self.pose_x
        pose.position.y = self.pose_y
        pose.position.z = self.pose_z
        pose_roll_euler = self.pose_roll
        pose_pitch_euler = self.pose_pitch
        pose_yaw_euler = self.pose_yaw

        quaternion = transformations.quaternion_from_euler(
            pose_roll_euler,
            pose_pitch_euler,
            pose_yaw_euler
        )

        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        return pose

    def wait_for_subscribers(self):
        i = 0
        while not is_shutdown() and (self.twist_connections == 0 or self.pose_connections == 0):
            if i == self.delay_wait_print:
                if self.twist_connections == 0:
                    loginfo(f"Waiting for subscriber to connect to {self.twist_publisher.name}")
                if self.pose_connections == 0:
                    loginfo(f"Waiting for subscriber to connect to {self.pose_publisher.name}")
            sleep(0.5)
            i += 1
            i = i % (self.delay_wait_print + 1)

        if is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

        self.is_teleop_ready = True

    def update(
        self, x, y, z, th,
        speed, turn, pose_x, pose_y, pose_z,
        pose_roll, pose_pitch, pose_yaw,
        pose_speed, pose_turn
    ):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.pose_z = pose_z
        self.pose_roll = pose_roll
        self.pose_pitch = pose_pitch
        self.pose_yaw = pose_yaw
        self.pose_speed = pose_speed
        self.pose_turn = pose_turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def to_default_pose(self):
        self.update(
            0, 0, 0, 0, self.speed, self.turn,
            self.pose_x, self.pose_y, self.pose_z,
            self.pose_roll, self.pose_pitch, self.pose_yaw,
            self.pose_speed, self.pose_turn
        )

    def stop(self):
        self.done = True
        self.to_default_pose()
        self.is_teleop_ready = False
        self.join()

    def on_key(self, msg):
        key = msg.data
        if key:
            self.handle_key(key)

    def handle_key(self, key):
        x = self.x
        y = self.y
        z = self.z
        th = self.th
        pose_x = self.pose_x
        pose_y = self.pose_y
        pose_z = self.pose_z
        pose_roll = self.pose_roll
        pose_pitch = self.pose_pitch
        pose_yaw = self.pose_yaw
        pose_speed = self.pose_speed
        pose_turn = self.pose_turn
        speed = self.speed
        turn = self.turn

        if key in moveBindings.keys():
            x = moveBindings[key][0]
            y = moveBindings[key][1]
            z = moveBindings[key][2]
            th = moveBindings[key][3]
        elif key in speedBindings.keys():
            speed = speed * speedBindings[key][0]
            turn = turn * speedBindings[key][1]
            x = 0
            y = 0
            z = 0
            th = 0
            loginfo(self.get_vels(speed, turn))

        elif key in poseBindings.keys():
            pose_x += pose_speed * poseBindings[key][0]
            pose_y += pose_speed * poseBindings[key][1]
            pose_z += pose_speed * poseBindings[key][2]
            pose_roll += pose_turn * poseBindings[key][3]
            pose_pitch += pose_turn * poseBindings[key][4]
            pose_yaw += pose_turn * poseBindings[key][5]
            x = 0
            y = 0
            z = 0
            th = 0

            loginfo(self.get_pose(
                pose_x, pose_y, pose_z,
                pose_roll, pose_pitch, pose_yaw
            ))
        elif key in speedPoseBindings.keys():
            pose_speed = pose_speed * speedPoseBindings[key][0]
            pose_turn = pose_turn * speedPoseBindings[key][1]
            x = 0
            y = 0
            z = 0
            th = 0

            loginfo(self.get_pose_vel(pose_speed, pose_turn))

        else:
            # Skip updating cmd_vel if key timeout and robot already
            # stopped.
            if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                return
            x = 0
            y = 0
            z = 0
            th = 0

        self.update(
            x, y, z, th,
            speed, turn, pose_x, pose_y, pose_z,
            pose_roll, pose_pitch, pose_yaw,
            pose_speed, pose_turn
        )

    @staticmethod
    def get_vels(speed, turn):
        return f"\tSpeed: {speed}\tTurn: {turn}"

    @staticmethod
    def get_pose_vel(pose_speed, pose_turn):
        return f"\tPose speed: {pose_speed}\tPose turn: {pose_turn}"

    @staticmethod
    def get_pose(pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw):
        return f"\tx: {pose_x}\ty {pose_y}\tz {pose_z}\troll {pose_roll}\tpitch {pose_pitch}\tyaw {pose_yaw}"

    def run(self):
        twist = Twist()

        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn
            pose = self.pose
            self.condition.release()

            # Publish
            self.twist_publisher.publish(twist)
            self.pose_publisher.publish(pose)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.to_default_pose()
        self.twist_publisher.publish(twist)
        self.pose_publisher.publish(self.pose)


if __name__ == "__main__":
    remote_teleop_thread = RemoteTeleopThread()
    try:
        remote_teleop_thread.wait_for_subscribers()
        remote_teleop_thread.to_default_pose()
        loginfo(msg)
        spin()
    except Exception as e:
        logerr(e)
    finally:
        remote_teleop_thread.stop()
