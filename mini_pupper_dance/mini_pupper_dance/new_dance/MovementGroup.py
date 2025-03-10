#
# Copyright 2024 MangDang (www.mangdang.net) 
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description: FPC(Flexible Programmable Choreography) APIs
#              There are 3 levels of APIs
#                 Level 1(for beginners): Simple APIs without input parameters
#                 Level 2(for makers): APIs with input parameters
#                 Level 3(for beyond): Samples delicately control the foot locations, move speed, and attitudes at each execution time.
#
import numpy as np
from .MovementScheme import Movements 

class MovementGroups:

    def __init__(self):
        self.MovementLib = []
        self.default_stand = [[[0.06,-0.05,-0.07]],[[0.06, 0.05,-0.07]],[[-0.06,-0.05,-0.07]],[[-0.06, 0.05,-0.07]]]
        
        # time of an excecution interval
        self.dt = 0.015
        
        # the limit parameters of different kinds of movements, you may try these predefined 
        self.vxcap = 0.5
        self.vycap = 0.5
        self.rowcap = 25
        self.pitchcap = 20
        self.yawcap = 30
        self.highcap = 0.04
        self.lowcap = 0.05
        self.legliftcap = 0.06
        
    def cap_limit(self, MAX, MIN, value):
        """define the movement limit to make sure motors' safety"""
        if value > MAX:
            num_modified = MAX
        elif value < MIN:
            num_modified = MIN
        else:
            num_modified = value
        return num_modified

########## The level1 simple APIs without input parameters ###########
 
    def stop(self, time = 1):
        """Return to the default natural standing position
        Args:
            time: let the robot be still in the defaut state for a certain period (unit: second)
        Returns:
        	Append the default standing position into MovementLib
        """
        if time <=0:
            time = self.dt
        interval = int(time / self.dt)
        dance_scheme = Movements('stop')
        dance_all_legs = self.default_stand
        dance_speed = [[0,0,0],[0,0,0]]        # speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]     # roll, pitch, yaw degree
        dance_scheme.setInterpolationNumber(interval)
        dance_scheme.setTransitionTic(70)
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
 
    def look_up(self):  
        """Set robot look up 20deg, you can change the deg parameter in this function.
        Returns:
        	Append the look up movement into MovementLib
        """    
        dance_scheme = Movements('look_up')
        dance_all_legs = self.default_stand
        dance_speed = [[0,0,0]]       # speed_x, speed_y, no_use
        dance_attitude = [[0,20,0],[0,10,0]]   # roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)     # append dance
        return self.MovementLib
 
    def look_down(self):
        """Set robot look down 20deg, you can change the deg parameter in this function.
        Returns:
        	Append the look down movement into MovementLib
        """ 
        dance_scheme = Movements('look_down')
        dance_all_legs = self.default_stand
        dance_speed = [[0,0,0]]       # speed_x, speed_y, no_use
        dance_attitude = [[0,-20,0]]  # roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib

    def look_right(self):
        """Set robot look right 30deg, you can change the deg parameter in this function.
        Returns:
        	Append the look right movement into MovementLib
        """
        dance_scheme = Movements('look_right')
        dance_all_legs = self.default_stand
        dance_speed = [[0,0,0]]       # speed_x, speed_y, no_use
        dance_attitude = [[0,0,30]]   # roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
 
    def look_left(self):
        """Set robot look left 30deg, you can change the deg parameter in this function.
        Returns:
        	Append the look left movement into MovementLib
        """
        dance_scheme = Movements('look_left')
        dance_all_legs = self.default_stand
        dance_speed = [[0,0,0]]       # speed_x, speed_y, no_use
        dance_attitude = [[0,0,-30]]  # roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
  
    def look_upperleft(self):
        """Set robot look up 20deg and look left 30deg, you can change the deg parameter in this function.
        Returns:
        	Append the look upperleft movement into MovementLib
        """
        dance_scheme = Movements('look_upperleft')
        dance_all_legs = self.default_stand
        dance_speed = [[0,0,0]]       # speed_x, speed_y, no_use
        dance_attitude = [[0,20,-30]] # roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib

    def look_upperright(self):
        """Set robot look up 20deg and look right 30deg, you can change the deg parameter in this function.
        Returns:
        	Append the look upperright movement into MovementLib
        """
        dance_scheme = Movements('look_upperright')
        dance_all_legs = self.default_stand
        dance_speed = [[0,0,0]]       # speed_x, speed_y, no_use
        dance_attitude = [[0,20,30]]  # roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
  
    def look_rightlower(self):
        """Set robot look down 20deg and look right 30deg, you can change the deg parameter in this function.
        Returns:
        	Append the look rightlower movement into MovementLib
        """
        dance_scheme = Movements('look_rightlower')
        dance_all_legs = self.default_stand
        dance_speed = [[0,0,0]]       # speed_x, speed_y, no_use
        dance_attitude = [[0,-20,30]] # roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
  
    def look_leftlower(self):
        """Set robot look down 20deg and look left 30deg, you can change the deg parameter in this function.
        Returns:
        	Append the look leftlower movement into MovementLib
        """
        dance_scheme = Movements('look_leftlower')
        dance_all_legs = self.default_stand
        dance_speed = [[0,0,0]]       # speed_x, speed_y, no_use
        dance_attitude = [[0,-20,-30]]# roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
                          
    def move_forward(self):
        """Set robot move forward as 0.15m/s, you can change the velocity parameter in this function.
        Returns:
        	Append the move forward movement into MovementLib
        """
        dance_scheme = Movements('move_forward')
        dance_all_legs = self.default_stand
        dance_speed = [[0.15,0,0]]    # speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]    # roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
     
    def move_backward(self):
        """Set robot move backward as 0.15m/s, you can change the velocity parameter in this function.
        Returns:
        	Append the move backward movement into MovementLib
        """
        dance_scheme = Movements('move_backward')
        dance_all_legs = self.default_stand
        dance_speed = [[-0.15,0,0]]   # speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]    # roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib

    def move_right(self):
        """Set robot move right as 0.15m/s, you can change the velocity parameter in this function.
        Returns:
        	Append the move right movement into MovementLib
        """
        dance_scheme = Movements('move_right')
        dance_all_legs = self.default_stand
        dance_speed = [[0,-0.15,0]]    # speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]     # roll, pitch, yaw rate
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
  
    def move_left(self):
        """Set robot move left as 0.15m/s, you can change the velocity parameter in this function.
        Returns:
        	Append the move left movement into MovementLib
        """
        dance_scheme = Movements('move_left')
        dance_all_legs = self.default_stand
        dance_speed = [[0,0.15,0]]    # speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]    # roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
   
    def move_leftfront(self):
        """Set robot move leftfront as 0.15*sqrt(2)m/s, you can change the velocity parameter in this function.
        Returns:
        	Append the move leftfront movement into MovementLib
        """
        dance_scheme = Movements('move_leftfront')
        dance_all_legs = self.default_stand
        dance_speed = [[0.15,0.15,0]] # speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]    # roll, pitch, yaw rate
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
    
    def move_rightfront(self):
        """Set robot move rightfront as 0.15*sqrt(2)m/s, you can change the velocity parameter in this function.
        Returns:
        	Append the move rightfront movement into MovementLib
        """
        dance_scheme = Movements('move_rightfront')
        dance_all_legs = self.default_stand
        dance_speed = [[0.15,-0.15,0]] # speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]     # roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
    
    def move_leftback(self):
        """Set robot move leftback as 0.15*sqrt(2)m/s, you can change the velocity parameter in this function.
        Returns:
        	Append the move leftback movement into MovementLib
        """
        dance_scheme = Movements('move_leftback')
        dance_all_legs = self.default_stand
        dance_speed = [[-0.15,0.15,0]]# speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]    # roll, pitch, yaw degree
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
 
    def move_rightback(self):
        """Set robot move rightback as 0.15*sqrt(2)m/s, you can change the velocity parameter in this function.
        Returns:
        	Append the move rightback movement into MovementLib
        """
        dance_scheme = Movements('move_rightback')
        dance_all_legs = self.default_stand
        dance_speed = [[-0.15,-0.15,0]]# speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]     # roll, pitch, yaw rate
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
        
 #----------- The simple APIs without input parameters END -----------#



 ########## The level2 APIs with input parameters ###########
        
    def head_move(self, pitch_deg = 0, yaw_deg = 0, time_uni = 1, time_acc = 1):
        """Turn the head of the robot to a certain degree
        Args: 
            Pitch_deg: the angle you want the robot's head to look up or down 
                        e.g. 20 ----> the pupper will lookup 20 degrees from pupper's own perspective
            yaw_deg: the angle you want the robot's head to look left or right 
                        e.g. 20 ----> the pupper will look right 20 degrees from pupper's own perspective
            time_acc: how long it takes to reach the desired angle (unit: second)
            time_uni: how long pupper will keep still at the desired pose (unit: second)
        
        Return:
            Append the head turning movement into the MovementLib
        """ 
        if time_uni <= 0:
            time_uni = self.dt
        if time_acc <=0:
            time_acc = self.dt
        interval_uni = int(time_uni / self.dt)
        interval_acc = int(time_acc / self.dt)
        modified_pitch = self.cap_limit(self.pitchcap, -self.pitchcap, pitch_deg)
        modified_yaw = self.cap_limit(self.yawcap, -self.yawcap, yaw_deg)
        dance_scheme = Movements('head_move')
        dance_all_legs = self.default_stand
        dance_speed = [[0,0,0]]        # speed_x, speed_y, no_use
        dance_attitude = [[0,pitch_deg,yaw_deg],[0,pitch_deg,yaw_deg]]     # roll, pitch, yaw degree
        dance_scheme.setInterpolationNumber(interval_uni)
        dance_scheme.setTransitionTic(interval_acc)
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
        
    def body_row(self, row_deg = 0, time_uni = 1, time_acc = 1):
        """Set the robot to tilt its body to a certain angle
        Args:
            row_deg: the desired angle you want the robot to tilt 
                    e.g. 10 ----> the pupper will tilt 10 degrees counterclockwise at it's own perspective
            time_acc: how long it takes to reach the desired angle (unit: second)
            time_uni: how long pupper will keep at the desired pose (unit: second)
        Return:
            Append the body row movement into the MovementLib
        """
        if time_uni <= 0:
            time_uni = self.dt
        if time_acc <=0:
            time_acc = self.dt
        interval_uni = int(time_uni / self.dt)
        interval_acc = int(time_acc / self.dt)
        modified_row = self.cap_limit(self.rowcap, -self.rowcap, row_deg)
        dance_scheme = Movements('body_row')
        dance_all_legs = self.default_stand
        dance_speed = [[0,0,0]]        # speed_x, speed_y, no_use
        dance_attitude = [[modified_row,0,0],[modified_row,0,0]]     # roll, pitch, yaw degree
        dance_scheme.setInterpolationNumber(interval_uni)
        dance_scheme.setTransitionTic(interval_acc)
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
    
    def gait_uni(self, v_x = 0, v_y = 0, time_uni = 1, time_acc = 1):
        """Let robot gait uniformly for a given time
        Args:
            v_x: the desired forward/back velocity (unit: m/s)
            v_y: the desired left/right velocity (unit: m/s)
            time_acc: how long it takes to accelerate from the previous speed to defined speed in this movement (unit: second)
            time_uni: how long the uniform movement will continue (unit second)
        Return:
            Append the uniform gait movement into the MovementLib
        """
        if time_uni <= 0:
            time_uni = self.dt
        if time_acc <=0:
            time_acc = self.dt
        interval_uni = int(time_uni / self.dt)
        interval_acc = int(time_acc / self.dt)
        modified_vx = self.cap_limit(self.vxcap, -self.vxcap, v_x)
        modified_vy = self.cap_limit(self.vycap, -self.vycap, v_y)
        dance_scheme = Movements('gait_uni')
        dance_all_legs = self.default_stand
        dance_speed = [[modified_vx,modified_vy,0],[modified_vx,modified_vy,0]]        # speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]     # roll, pitch, yaw degree
        dance_scheme.setInterpolationNumber(interval_uni)
        dance_scheme.setTransitionTic(interval_acc)
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib   
        
    def height_move(self, ht = 0, time_uni = 1, time_acc = 1):
        """Let robot descend or ascend a given height
        Args:
            ht: the distance you want the robot to ascend or descend 
                e.g. ht = 0.02 ----> let pupper ascend 0.02m
            time_acc: how long it takes to ascend or descend (unit: second)
            time_uni: how long pupper will hold the position after ascending or descending (unit:second)
        Return:
            Append the height movement into the MovementLib
        """
        if time_uni <= 0:
            time_uni = self.dt
        if time_acc <=0:
            time_acc = self.dt
        interval_uni = int(time_uni / self.dt)
        interval_acc = int(time_acc / self.dt)
        modified_ht = self.cap_limit(self.highcap, -self.lowcap, ht)
        dance_scheme = Movements('height_move')
        dance_all_legs = [
            [[ 0.06,-0.05,-0.07-modified_ht],[ 0.06,-0.05,-0.07-modified_ht]],
            [[ 0.06, 0.05,-0.07-modified_ht],[ 0.06, 0.05,-0.07-modified_ht]],
            [[-0.06,-0.05,-0.07-modified_ht],[-0.06,-0.05,-0.07-modified_ht]],
            [[-0.06, 0.05,-0.07-modified_ht],[-0.06, 0.05,-0.07-modified_ht]]
        ]
        dance_speed = [[0,0,0]]        # speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]     # roll, pitch, yaw degree
        dance_scheme.setTransitionTic(interval_acc)
        dance_scheme.setInterpolationNumber(interval_uni) 
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
        
    def foreleg_lift(self, leg_index = 'left', ht = 0.01, time_uni = 1, time_acc = 1):
        """lift one foreleg by a certain height
        Args:
            ht: the height you want the leg to lift by 
                e.g. ht = 0.02 ----> lift up the leg by 0.02m
            leg_index: 'left' or 'right', indicating which leg to lift
            time_acc: how long it takes to lift the leg (unit: second)
            time_uni: how long pupper will keep still at the leg lifted pose (unit: second)
        Return: 
            Append the liftup movement into the MovementLib
        """
        if time_uni <= 0:
            time_uni = self.dt
        if time_acc <=0:
            time_acc = self.dt
        interval_uni = int(time_uni / self.dt)
        interval_acc = int(time_acc / self.dt)
        modified_ht = self.cap_limit(self.legliftcap, 0, ht)
        dance_scheme = Movements('foreleg')
        dance_all_legs = []
        leg_1 = [
            [[0.12,-0.06,-0.07+modified_ht],[0.12,-0.06,-0.07+modified_ht]],
            [[0.06, 0.01,-0.07],[0.06, 0.01,-0.07]],
            [[-0.06,-0.05,-0.09],[-0.06,-0.05,-0.09]],
            [[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07]]
        ]
        leg_2 = [
            [[0.06,-0.01,-0.07],[0.06,-0.01,-0.07]],
            [[0.12, 0.06,-0.07+modified_ht],[0.12, 0.06,-0.07+modified_ht]],
            [[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07]],
            [[-0.06, 0.05,-0.09],[-0.06, 0.05,-0.09]]
        ]
        if leg_index == 'right':
            dance_all_legs = leg_1
        elif leg_index == 'left':
            dance_all_legs = leg_2
        dance_speed = [[0,0,0]]        # speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]     # roll, pitch, yaw degree
        dance_scheme.setTransitionTic(interval_acc)
        dance_scheme.setInterpolationNumber(interval_uni)
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib
        
    def backleg_lift(self, leg_index = 'left', ht = 0.01, time_uni = 1, time_acc = 1):
        """lift one backleg by a certain height
        Args:
            ht: the height you wan the leg to lift by 
                e.g. ht = 0.02 ----> lift up the leg by 0.02m
            leg_index: 'left' or 'right', indicating which leg to lift
            time_acc: how long it takes to lift the leg unit: second (unit: second)
            time_uni: how long pupper will keep still at the leg lifted pose (unit:second)
        Return: 
            Append the liftup movement into the MovementLib
        """
        if time_uni <= 0:
            time_uni = self.dt
        if time_acc <=0:
            time_acc = self.dt
        interval_uni = int(time_uni / self.dt)
        interval_acc = int(time_acc / self.dt)
        modified_ht = self.cap_limit(self.legliftcap, 0, ht)
        dance_scheme = Movements('backleg')
        dance_all_legs = []
        leg_1 = [
            [[0.06,-0.05,-0.09],[0.06,-0.05,-0.09]],
            [[0.06, 0.05,-0.07],[0.06, 0.05,-0.07]],
            [[-0.08,-0.08,-0.07+modified_ht],[-0.08,-0.08,-0.07+modified_ht]],
            [[-0.06, 0.01,-0.07],[-0.06, 0.01,-0.07]]
        ]
        leg_2 = [
            [[0.06,-0.05,-0.07],[0.06,-0.05,-0.07]],
            [[0.06, 0.05,-0.09],[0.06, 0.05,-0.09]],
            [[-0.06,-0.01,-0.07],[-0.06,-0.01,-0.07]],
            [[-0.08, 0.08,-0.07+modified_ht],[-0.08, 0.08,-0.07+modified_ht]]
        ]
        if leg_index == 'right':
            dance_all_legs = leg_1
        elif leg_index == 'left':
            dance_all_legs = leg_2
        dance_speed = [[0,0,0]]        # speed_x, speed_y, no_use
        dance_attitude = [[0,0,0]]     # roll, pitch, yaw degree
        dance_scheme.setTransitionTic(interval_acc)
        dance_scheme.setInterpolationNumber(interval_uni)
        dance_scheme.setAllSequence(dance_all_legs,dance_speed,dance_attitude)
        self.MovementLib.append(dance_scheme)      # append dance
        return self.MovementLib

#----------- The level2 APIs with input parameters END -----------#




##### The level3 samples to DIY complicated movement   #######

    """ These Samples delicately control the foot locations, move speed and attitudes at each execution times,
    enabling more complicated movement based on your demands. By defaut, no arguments are needed, but you can 
    modify them on your own."""

    def body_cycle(self):
        """ This movement enables the pupper to draw a circle with it's body center in the x-y plane while keeping 
        it's origional orientation of the body.
        Params:
            Radius: the radius of the circle trajectory (unit: meter)
        """
        dance_scheme = Movements('body_cycle')
        Radius = 0.04

        dance_all_legs = []
        dance_all_legs.append([
                               [ 0.06+np.cos(1*22.5*np.pi/180)*Radius,-0.05+np.sin(1*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(2*22.5*np.pi/180)*Radius,-0.05+np.sin(2*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(3*22.5*np.pi/180)*Radius,-0.05+np.sin(3*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(4*22.5*np.pi/180)*Radius,-0.05+np.sin(4*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(5*22.5*np.pi/180)*Radius,-0.05+np.sin(5*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(6*22.5*np.pi/180)*Radius,-0.05+np.sin(6*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(7*22.5*np.pi/180)*Radius,-0.05+np.sin(7*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(8*22.5*np.pi/180)*Radius,-0.05+np.sin(8*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(9*22.5*np.pi/180)*Radius,-0.05+np.sin(9*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(10*22.5*np.pi/180)*Radius,-0.05+np.sin(10*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(11*22.5*np.pi/180)*Radius,-0.05+np.sin(11*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(12*22.5*np.pi/180)*Radius,-0.05+np.sin(12*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(13*22.5*np.pi/180)*Radius,-0.05+np.sin(13*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(14*22.5*np.pi/180)*Radius,-0.05+np.sin(14*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(15*22.5*np.pi/180)*Radius,-0.05+np.sin(15*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(16*22.5*np.pi/180)*Radius,-0.05+np.sin(16*22.5*np.pi/180)*Radius,-0.07],
                               ])# leg1(front right) foot locations
        
        dance_all_legs.append([
                               [ 0.06+np.cos(1*22.5*np.pi/180)*Radius,0.05+np.sin(1*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(2*22.5*np.pi/180)*Radius,0.05+np.sin(2*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(3*22.5*np.pi/180)*Radius,0.05+np.sin(3*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(4*22.5*np.pi/180)*Radius,0.05+np.sin(4*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(5*22.5*np.pi/180)*Radius,0.05+np.sin(5*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(6*22.5*np.pi/180)*Radius,0.05+np.sin(6*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(7*22.5*np.pi/180)*Radius,0.05+np.sin(7*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(8*22.5*np.pi/180)*Radius,0.05+np.sin(8*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(9*22.5*np.pi/180)*Radius,0.05+np.sin(9*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(10*22.5*np.pi/180)*Radius,0.05+np.sin(10*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(11*22.5*np.pi/180)*Radius,0.05+np.sin(11*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(12*22.5*np.pi/180)*Radius,0.05+np.sin(12*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(13*22.5*np.pi/180)*Radius,0.05+np.sin(13*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(14*22.5*np.pi/180)*Radius,0.05+np.sin(14*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(15*22.5*np.pi/180)*Radius,0.05+np.sin(15*22.5*np.pi/180)*Radius,-0.07],
                               [ 0.06+np.cos(16*22.5*np.pi/180)*Radius,0.05+np.sin(16*22.5*np.pi/180)*Radius,-0.07],
                               ])# leg2(front left) foot locations
        
        dance_all_legs.append([
                               [ -0.06+np.cos(1*22.5*np.pi/180)*Radius,-0.05+np.sin(1*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(2*22.5*np.pi/180)*Radius,-0.05+np.sin(2*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(3*22.5*np.pi/180)*Radius,-0.05+np.sin(3*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(4*22.5*np.pi/180)*Radius,-0.05+np.sin(4*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(5*22.5*np.pi/180)*Radius,-0.05+np.sin(5*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(6*22.5*np.pi/180)*Radius,-0.05+np.sin(6*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(7*22.5*np.pi/180)*Radius,-0.05+np.sin(7*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(8*22.5*np.pi/180)*Radius,-0.05+np.sin(8*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(9*22.5*np.pi/180)*Radius,-0.05+np.sin(9*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(10*22.5*np.pi/180)*Radius,-0.05+np.sin(10*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(11*22.5*np.pi/180)*Radius,-0.05+np.sin(11*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(12*22.5*np.pi/180)*Radius,-0.05+np.sin(12*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(13*22.5*np.pi/180)*Radius,-0.05+np.sin(13*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(14*22.5*np.pi/180)*Radius,-0.05+np.sin(14*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(15*22.5*np.pi/180)*Radius,-0.05+np.sin(15*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(16*22.5*np.pi/180)*Radius,-0.05+np.sin(16*22.5*np.pi/180)*Radius,-0.07],
                               ])# leg3(back right) foot locations
        
        dance_all_legs.append([
                               [ -0.06+np.cos(1*22.5*np.pi/180)*Radius,0.05+np.sin(1*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(2*22.5*np.pi/180)*Radius,0.05+np.sin(2*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(3*22.5*np.pi/180)*Radius,0.05+np.sin(3*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(4*22.5*np.pi/180)*Radius,0.05+np.sin(4*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(5*22.5*np.pi/180)*Radius,0.05+np.sin(5*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(6*22.5*np.pi/180)*Radius,0.05+np.sin(6*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(7*22.5*np.pi/180)*Radius,0.05+np.sin(7*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(8*22.5*np.pi/180)*Radius,0.05+np.sin(8*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(9*22.5*np.pi/180)*Radius,0.05+np.sin(9*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(10*22.5*np.pi/180)*Radius,0.05+np.sin(10*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(11*22.5*np.pi/180)*Radius,0.05+np.sin(11*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(12*22.5*np.pi/180)*Radius,0.05+np.sin(12*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(13*22.5*np.pi/180)*Radius,0.05+np.sin(13*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(14*22.5*np.pi/180)*Radius,0.05+np.sin(14*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(15*22.5*np.pi/180)*Radius,0.05+np.sin(15*22.5*np.pi/180)*Radius,-0.07],
                               [ -0.06+np.cos(16*22.5*np.pi/180)*Radius,0.05+np.sin(16*22.5*np.pi/180)*Radius,-0.07],
                               ])# leg4(back left) foot locations

        dance_speed    = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],]
        dance_attitude = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[ 0,0,0],[0,0,0],[0,0,0]]    # roll, pitch, yaw
        
        dance_scheme.setInterpolationNumber(15)
        dance_scheme.setLegsSequence(dance_all_legs,"Multiple",3)
        dance_scheme.setAttitudeSequence(dance_attitude,"Multiple",3)
        dance_scheme.setSpeedSequence(dance_speed,"Multiple",3)
        self.MovementLib.append(dance_scheme)      # append dance

        return self.MovementLib

    def head_ellipse(self):
        """ This movement enables the pupper to draw an ellipse-shaped trajectory with it's head by edting the orientations of
         the body. The maximum pitching angle is 1/9 π and the maximum yawing angle is 1/6 π 
        """
        dance_scheme = Movements('head_ellipse')

        dance_all_legs = []
        dance_all_legs.append([[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],[ 0.06,-0.05,-0.07],])# leg1
        dance_all_legs.append([[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],[ 0.06, 0.05,-0.07],])# leg2
        dance_all_legs.append([[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],[-0.06,-0.05,-0.07],])# leg3
        dance_all_legs.append([[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],[-0.06, 0.05,-0.07],])# leg4

        dance_speed    = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],] 
        dance_attitude = [[0,20*1.414/2,30*1.414/2],[0,20*0.38268,30*0.92388],[0,0,30],[0,-20*0.38268,30*0.92388],[0,-20*1.414/2,30*1.414/2],[0,-20*0.92388,30*0.38268],[0,-20,0],[0,-20*0.92388,-30*0.38268],[0,-20*1.414/2,-30*1.414/2],[0,-20*0.38268,-30*0.92388],[0,0,-30],[0,20*0.38268,-30*0.92388],[0,20*1.414/2,-30*1.414/2],[0,20*0.92388,-30*0.38268],[0,20,0],[0,20*0.92388,30*0.38268],]         # roll, pitch, yaw
        
        dance_scheme.setInterpolationNumber(15)
        dance_scheme.setLegsSequence(dance_all_legs,"Multiple",3)
        dance_scheme.setAttitudeSequence(dance_attitude,"Multiple",3)
        dance_scheme.setSpeedSequence(dance_speed,"Multiple",3)
        self.MovementLib.append(dance_scheme)      # append dance

        return self.MovementLib

#----------- The level3 samples to DIY complicated movement END END -----------#  
