import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from vision_msgs.msg import Detection2DArray
import math

class OAK_DETECT(Node):
    def __init__(self):
        super().__init__('oak_detect')
        
        self.activated = False
        
        self.timer_freq = 15 #hz
        
        self.image_height = 300
        self.image_width = 300
        
        self.robot_roll = 0
        self.robot_pitch = 0
        self.robot_yaw = 0
        
        self.robot_rpy_upper_bounds = [0.42, 0.42, 0.42]
        self.robot_rpy_lower_bounds = [-0.42, -0.42, -0.42]
        
        self.pose_msg = Pose()
        self.detections = None
        
        self.detection_subscriber = self.create_subscription(
                            Detection2DArray,
                            '/color/mobilenet_detections',
                            self.detection_callback,
                            10)
        self.pose_publisher = self.create_publisher(Pose, '/body_pose', 10)
        self.timer = self.create_timer(1.0 / self.timer_freq, self.timer_callback)
    
    def limit_in_bounds(self, value, upper_bound, lower_bound):
        if(value > upper_bound):
            return upper_bound
        elif(value < lower_bound):
            return lower_bound
        else:
            return value
    
    def limit_robot_rpy(self):
        # roll = euler_list[0]
        # pitch = euler_list[1]
        # yaw = euler_list[2]
        
        self.robot_roll = self.limit_in_bounds(self.robot_roll,\
                                               self.robot_rpy_upper_bounds[0],\
                                               self.robot_rpy_lower_bounds[0])
        self.robot_pitch = self.limit_in_bounds(self.robot_pitch,\
                                                self.robot_rpy_upper_bounds[1],\
                                                self.robot_rpy_lower_bounds[1])
        self.robot_yaw = self.limit_in_bounds(self.robot_yaw,\
                                              self.robot_rpy_upper_bounds[2],\
                                              self.robot_rpy_lower_bounds[2])
            
    def quaternion_from_euler(self, euler_list):
        # roll = euler_list[0]
        # pitch = euler_list[1]
        # yaw = euler_list[2]
        
        w = math.cos(euler_list[2] * 0.5) * math.cos(euler_list[1] * 0.5) *\
            math.cos(euler_list[0] * 0.5) + math.sin(euler_list[2] * 0.5) *\
            math.sin(euler_list[1] * 0.5) * math.sin(euler_list[0] * 0.5)
        x = math.cos(euler_list[2] * 0.5) * math.cos(euler_list[1] * 0.5) *\
            math.sin(euler_list[0] * 0.5) - math.sin(euler_list[2] * 0.5) *\
            math.sin(euler_list[1] * 0.5) * math.cos(euler_list[0] * 0.5)
        y = math.sin(euler_list[2] * 0.5) * math.cos(euler_list[1] * 0.5) *\
            math.sin(euler_list[0] * 0.5) + math.cos(euler_list[2] * 0.5) *\
            math.sin(euler_list[1] * 0.5) * math.cos(euler_list[0] * 0.5)
        z = math.sin(euler_list[2] * 0.5) * math.cos(euler_list[1] * 0.5) *\
            math.cos(euler_list[0] * 0.5) - math.cos(euler_list[2] * 0.5) *\
            math.sin(euler_list[1] * 0.5) * math.sin(euler_list[0] * 0.5)
        
        return [w, x, y, z]
    
    def pack_msg(self):
        rpy_data = [self.robot_roll, self.robot_pitch, self.robot_yaw]
        [w, x, y, z] = self.quaternion_from_euler(rpy_data)
        
        self.pose_msg.orientation.w = w
        self.pose_msg.orientation.x = x
        self.pose_msg.orientation.y = y
        self.pose_msg.orientation.z = z
    
    def calc_and_pub_pose(self, target_id, detection_data):
        # target id list of depthai_examples/launch/mobile_publisher.launch.py:
        # 0: background
        # 1: aeroplane
        # 2: bicycle
        # 3: bird
        # 4: boat
        # 5: bottle
        # 6: bus
        # 7: car
        # 8: cat
        # 9: chair
        # 10: cow
        # 11: dining table
        # 12: dog
        # 13: horse
        # 14: motorbike
        # 15: person
        # 16: potted plant
        # 17: sheep
        # 18: sofa
        # 19: train
        # 20: tv monitor
        
        droll = 0
        dpitch = 0
        dyaw = 0
        
        for detection in detection_data:
            bbox = detection.bbox
            if(detection.id == str(target_id)):
                droll = 0
                dpitch = self.limit_in_bounds(-0.0002 * (self.image_height / 2.0 - bbox.center.position.y), 0.01, -0.01)
                dyaw = self.limit_in_bounds(0.0002 * (self.image_width / 2.0 - bbox.center.position.x), 0.01, -0.01)
        
        self.robot_roll += droll
        self.robot_pitch += dpitch
        self.robot_yaw += dyaw
        
        self.limit_robot_rpy()
        
        self.pack_msg()
        
        self.pose_publisher.publish(self.pose_msg)
        
    def timer_callback(self):
        if(self.activated):
            self.calc_and_pub_pose(5, self.detections)
    
    def detection_callback(self, msg):
        self.detections = msg.detections
        self.activated = True

def main(args=None):
    rclpy.init(args = args)
    oak_detect_node = OAK_DETECT()
    rclpy.spin(oak_detect_node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
