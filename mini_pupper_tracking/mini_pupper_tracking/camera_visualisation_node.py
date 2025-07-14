import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point
from stanford_controller.Config import Configuration
import math
from mini_pupper_interfaces.msg import TrackingArray, Tracking

def to_point(p):
    return Point(x=p[0], y=p[1], z=p[2])

class CameraVisualisationNode(Node):
    def __init__(self):
        super().__init__('camera_visualisation_node')
        self.publisher = self.create_publisher(Marker, '/camera_fov', 10)
        self.config = Configuration()

        # Camera FOV
        self.camera_position = (0.070, 0.000, 0.035)
        self.range = 2.0
        self.fov_deg = 62.2
        self.vertical_fov_deg = 48.8
        self.fov_rad = math.radians(self.fov_deg)
        self.vertical_fov_rad = math.radians(self.vertical_fov_deg)

        self.fov_timer = self.create_timer(1.0, self.publish_fov)

        self.people_subscriber = self.create_subscription(TrackingArray, "/tracking_array", self.people_callback, 10)
        self.people_points = [Point()]
        self.people_timer = self.create_timer(self.config.dt, self.publish_people)

    def people_callback(self, msg):
        points = []
        for track in msg.tracks:
            x = track.center_x
            y = track.top_y
            A = track.bounding_area

            # Angular position within FOV
            angle_x = (x - 0.5) * self.fov_rad # centre is at 0.5
            angle_y = (0.5 - y) * self.vertical_fov_rad # invert Y because top is 0

            # Depth approximation
            depth = min(2.5, 0.3 / math.pow(max(A, 0.001), 2.5))

            # In camera frame (FOV points in +X direction)
            X = depth
            Y = depth * math.tan(angle_x)
            Z = depth * math.tan(angle_y)

            point = to_point([X, Y, Z])
            points.append(point)
        
        self.people_points = points

    def publish_people(self):
        people = Marker()
        people.ns = 'camera_fov'
        people.id = 2
        people.type = Marker.POINTS
        people.action = Marker.ADD
        people.header.frame_id = 'base_link'
        people.pose = self.camera_pose()
        people.scale.x = 0.04
        people.scale.y = 0.04
        people.scale.z = 0.04
        people.header.stamp = self.get_clock().now().to_msg()
        people.color.r = 1.0
        people.color.g = 0.0 
        people.color.b = 0.0
        people.color.a = 1.0  
        people.lifetime.sec = int(0)
        people.lifetime.nanosec = int(self.config.dt * (10**9))
        people.frame_locked = True  
        people.points = self.people_points

        self.publisher.publish(people)

    def publish_fov(self):
        edges = Marker()
        edges.ns = 'camera_fov'
        edges.id = 0
        edges.type = Marker.LINE_LIST
        edges.action = Marker.ADD
        edges.header.frame_id = 'base_link'
        edges.pose = self.camera_pose()
        edges.scale.x = 0.005
        edges.header.stamp = self.get_clock().now().to_msg()
        edges.color.r = 0.0
        edges.color.g = 0.0 
        edges.color.b = 1.0
        edges.color.a = 0.5  
        edges.lifetime.sec = int(0)
        edges.lifetime.nanosec = int(0)
        edges.frame_locked = True  
        edges.points, fov_vertices = self.compute_fov_lines()

        vertices = Marker()
        vertices.ns = 'camera_fov'
        vertices.id = 1
        vertices.type = Marker.POINTS
        vertices.action = Marker.ADD
        vertices.header.frame_id = 'base_link'
        vertices.pose = self.camera_pose()
        vertices.scale.x = 0.04
        vertices.scale.y = 0.04
        vertices.scale.z = 0.04
        vertices.header.stamp = self.get_clock().now().to_msg()
        vertices.color.r = 0.0
        vertices.color.g = 1.0 
        vertices.color.b = 0.0
        vertices.color.a = 0.6
        vertices.lifetime.sec = int(0)
        vertices.lifetime.nanosec = int(0)
        vertices.frame_locked = True  
        vertices.points = fov_vertices

        self.publisher.publish(edges)
        self.publisher.publish(vertices)
    
    def compute_fov_lines(self):

        r = self.range
        w = r * math.tan(self.fov_rad / 2)
        h = r * math.tan(self.vertical_fov_rad / 2)

        apex = (0.0, 0.0, 0.0)

        # FOV pointing in +X direction
        base = [
            (r,  w,  h),  # top right
            (r,  w, -h),  # bottom right  
            (r, -w, -h),  # bottom left
            (r, -w,  h)   # top left
        ]

        points = []

        fov_vertices = [
            to_point((0.0, 0.0, 0.0)),  # Apex
            to_point((r,  w,  h)),      # Top right
            to_point((r,  w, -h)),      # Bottom right
            to_point((r, -w, -h)),      # Bottom left
            to_point((r, -w,  h))       # Top left
        ]

        # Lines from apex to base corners
        for corner in base:
            points.append(to_point(apex))
            points.append(to_point(corner))

        # Lines around base rectangle (closing the square)
        for i in range(4):
            p1 = base[i]
            p2 = base[(i + 1) % 4]
            points.append(to_point(p1))
            points.append(to_point(p2))

        return points, fov_vertices

    def camera_pose(self):
        pose = Pose()
        pose.position.x = self.camera_position[0]
        pose.position.y = self.camera_position[1]
        pose.position.z = self.camera_position[2]
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        return pose

def main():
    rclpy.init()
    node = CameraVisualisationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
