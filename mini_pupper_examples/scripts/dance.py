#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import math
import time
import sys
import os

class dance_demo:
    def __init__(self):
        rospy.init_node('dance_demo',anonymous=True)
        self.r = rospy.Rate(100)
        self.r.sleep()
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.dance_config_path = rospy.get_param('~dance_config_path')
        sys.path.append(self.dance_config_path)
        rospy.loginfo(str(self.dance_config_path))
        rospy.loginfo(str(sys.path))
        self.dance_config_name = ' '
        self.commands = []
        self.ready_to_dance = 0
        self.dance_config_sub = rospy.Subscriber("/dance_config", String, self.dance_config_callback, queue_size = 10)
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        self.pose_publisher = rospy.Publisher('target_body_pose', Pose, queue_size=100)

    def dance_config_callback(self,data):
        self.dance_config_name = str(data.data)
        self.dance_config = __import__(self.dance_config_name)
        self.commands = self.dance_config.dance_commands
        self.commands.insert(0,'stop')
        self.commands.append('stop')
        self.ready_to_dance = 1

    def dance(self):
        while((self.dance_config_name == ' ' or not self.ready_to_dance) and not rospy.is_shutdown()):
            pass
        for command in self.commands:
            velocity_cmd = Twist()
            pose_cmd = Pose()
            self.roll = 0
            self.pitch = 0
            self.yaw = 0
            if(rospy.is_shutdown()):
                continue

            if(command == 'move_forward'):
                rospy.loginfo('executing command: '+str(command))
                velocity_cmd.linear.x = 0.5
                self.velocity_publisher.publish(velocity_cmd)
                
            elif(command == 'move_backward'):
                rospy.loginfo('executing command: '+str(command))
                velocity_cmd.linear.x = -0.5
                self.velocity_publisher.publish(velocity_cmd)
                
            elif(command == 'move_left'):
                rospy.loginfo('executing command: '+str(command))
                velocity_cmd.linear.y = 0.5
                self.velocity_publisher.publish(velocity_cmd)
                
            elif(command == 'move_right'):
                rospy.loginfo('executing command: '+str(command))
                velocity_cmd.linear.y = -0.5
                self.velocity_publisher.publish(velocity_cmd)
                
            elif(command == 'turn_left'):
                rospy.loginfo('executing command: '+str(command))
                velocity_cmd.angular.z = 1.0
                self.velocity_publisher.publish(velocity_cmd)
                
            elif(command == 'turn_right'):
                rospy.loginfo('executing command: '+str(command))
                velocity_cmd.angular.z = -1.0
                self.velocity_publisher.publish(velocity_cmd)
                
            elif(command == 'look_up'):
                rospy.loginfo('executing command: '+str(command))
                self.pitch = -0.3
                cy=math.cos(self.yaw*0.5)
                sy=math.sin(self.yaw*0.5)
                cp=math.cos(self.pitch*0.5)
                sp=math.sin(self.pitch*0.5)
                cr =math.cos(self.roll * 0.5)
                sr =math.sin(self.roll * 0.5)

                pose_cmd.orientation.w= cy * cp * cr + sy * sp * sr
                pose_cmd.orientation.x = cy * cp * sr - sy * sp * cr
                pose_cmd.orientation.y = sy * cp * sr + cy * sp * cr
                pose_cmd.orientation.z = sy * cp * cr - cy * sp * sr
                self.pose_publisher.publish(pose_cmd)
                
            elif(command == 'look_down'):
                rospy.loginfo('executing command: '+str(command))
                self.pitch = 0.3
                cy=math.cos(self.yaw*0.5)
                sy=math.sin(self.yaw*0.5)
                cp=math.cos(self.pitch*0.5)
                sp=math.sin(self.pitch*0.5)
                cr =math.cos(self.roll * 0.5)
                sr =math.sin(self.roll * 0.5)

                pose_cmd.orientation.w= cy * cp * cr + sy * sp * sr
                pose_cmd.orientation.x = cy * cp * sr - sy * sp * cr
                pose_cmd.orientation.y = sy * cp * sr + cy * sp * cr
                pose_cmd.orientation.z = sy * cp * cr - cy * sp * sr
                self.pose_publisher.publish(pose_cmd)
                
            elif(command == 'look_left'):
                rospy.loginfo('executing command: '+str(command))
                self.yaw = 0.3
                cy=math.cos(self.yaw*0.5)
                sy=math.sin(self.yaw*0.5)
                cp=math.cos(self.pitch*0.5)
                sp=math.sin(self.pitch*0.5)
                cr =math.cos(self.roll * 0.5)
                sr =math.sin(self.roll * 0.5)

                pose_cmd.orientation.w= cy * cp * cr + sy * sp * sr
                pose_cmd.orientation.x = cy * cp * sr - sy * sp * cr
                pose_cmd.orientation.y = sy * cp * sr + cy * sp * cr
                pose_cmd.orientation.z = sy * cp * cr - cy * sp * sr
                self.pose_publisher.publish(pose_cmd)
                
            elif(command == 'look_right'):
                rospy.loginfo('executing command: '+str(command))
                self.yaw = -0.3
                cy=math.cos(self.yaw*0.5)
                sy=math.sin(self.yaw*0.5)
                cp=math.cos(self.pitch*0.5)
                sp=math.sin(self.pitch*0.5)
                cr =math.cos(self.roll * 0.5)
                sr =math.sin(self.roll * 0.5)

                pose_cmd.orientation.w= cy * cp * cr + sy * sp * sr
                pose_cmd.orientation.x = cy * cp * sr - sy * sp * cr
                pose_cmd.orientation.y = sy * cp * sr + cy * sp * cr
                pose_cmd.orientation.z = sy * cp * cr - cy * sp * sr
                self.pose_publisher.publish(pose_cmd)
            
            elif(command == 'stop'):
                rospy.loginfo('executing command: '+str(command))
                
                cy=math.cos(self.yaw*0.5)
                sy=math.sin(self.yaw*0.5)
                cp=math.cos(self.pitch*0.5)
                sp=math.sin(self.pitch*0.5)
                cr =math.cos(self.roll * 0.5)
                sr =math.sin(self.roll * 0.5)

                pose_cmd.orientation.w= cy * cp * cr + sy * sp * sr
                pose_cmd.orientation.x = cy * cp * sr - sy * sp * cr
                pose_cmd.orientation.y = sy * cp * sr + cy * sp * cr
                pose_cmd.orientation.z = sy * cp * cr - cy * sp * sr
            
                self.velocity_publisher.publish(velocity_cmd)
                self.pose_publisher.publish(pose_cmd)
                
            elif(command == 'stay'):
                rospy.loginfo('executing command: '+str(command))
                
            else:
                rospy.logwarn('wrong command: '+str(command))
            
            time.sleep(self.dance_config.interval_time)

if __name__ == "__main__":
    lets_dance = dance_demo()
    lets_dance.dance()
