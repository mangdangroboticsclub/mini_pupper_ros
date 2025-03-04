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
# Description: Basic functions to support FPC(Flexible Programmable Choreography) APIs
#
import numpy as np

LocationStanding = [[ 0.06,0.06,-0.06,-0.06],[-0.05, 0.05,-0.05,0.05],[ -0.07,-0.07,-0.07,-0.07]]
SpeedStanding = [0,0,0]
AttitudeStanding = [0,0,0]

DeltLocationMax = 0.0005 #unit m
DeltSpeedMax = 0.0025 #unit m/s
DeltAttitudeMax = 1 #unit degree

AttitudeMinMax = [[-20,20],[-20,20],[-60,60]]

class SequenceInterpolation:
    """
    this class used to set sequence interpolation, such as legs_location,speed and attitude
    """
    def __init__(self,name,dimension):
        self.Name = name
        self.Dimension = dimension
        # default value is 70
        self.InterpolationNumber = 70

        self.ExecuteTick = 0
        self.SequenceExecuteCounter = 0
        self.ExecuteCounterMax = 1
        self.PhaseNumberMax = 1
        self.SequencePoint = [[0,0,0]]

        # interpolation point data 
        self.PointPhaseStart = 0
        self.PointPhaseStop = 1
        self.TnterpolationDelt = [0,0,0]
        self.PointNow = [0,0,0]                 
        self.PointPrevious = [0,0,0]

    def setCycleType(self,cycle_type,cycle_index):
        """set cycle type ,include Forever , Multiple, or single,the cycle_index is the loop times
        
        parameter: cycle_type,cycle_index
        """
        if cycle_type == 'Forever':
            self.SequenceExecuteCounter = 9999
        elif cycle_type == 'Multiple':
            self.SequenceExecuteCounter = cycle_index
        else:
            self.SequenceExecuteCounter = 1
        self.ExecuteCounterMax = cycle_index

        return True
        
    def setInterpolationNumber(self,interpolation_number):
        self.InterpolationNumber = interpolation_number
        return True
        
    def setSequencePoint(self,sequence):
        """this function is to init the sequence
        
        Parameters: sequence
        ---------  
        Returns: True
        """
        self.SequencePoint = sequence
        self.PhaseNumberMax = len(sequence)
        
        # init now and pre point phase
        for xyz in range(self.Dimension):
        
            self.PointNow[xyz] = sequence[0][xyz] 
            self.PointPrevious[xyz] = sequence[0][xyz] 
            
        # init start point phase
        self.PointPhaseStart  = 0
        # init stop point phase
        self.PointPhaseStop  = self.PointPhaseStart + 1
        if self.PointPhaseStop >= len(sequence):
            self.PointPhaseStop = self.PointPhaseStart
        #init the first diff
        point_start = sequence[self.PointPhaseStart]
        point_stop = sequence[self.PointPhaseStop]
        
        for xyz in range(self.Dimension):
            
            diff =   point_stop[xyz] - point_start[xyz]
              
            self.TnterpolationDelt[xyz] = diff/self.InterpolationNumber

        return True

    def updatePointPhase(self):
        """this function is to update the point phase ,include start phase and stop phase
        
        Parameters: Null
        ---------  
        Returns: True
        """
        # update start point phase
        self.PointPhaseStart  = self.PointPhaseStart + 1
        if self.PointPhaseStart >= self.PhaseNumberMax:
            if self.SequenceExecuteCounter >0:
                self.PointPhaseStart = 0
            else:
                self.SequenceExecuteCounter = 0
                self.PointPhaseStart = self.PointPhaseStart - 1

        # update stop point phase
        self.PointPhaseStop  = self.PointPhaseStart + 1
        if self.PointPhaseStop >= self.PhaseNumberMax:
            self.SequenceExecuteCounter = self.SequenceExecuteCounter - 1
            if self.SequenceExecuteCounter >0:
                self.PointPhaseStop = 0
            else:
                self.SequenceExecuteCounter = 0
                self.PointPhaseStop = self.PointPhaseStop - 1
            
        return True

    def updateInterpolationDelt(self):
        """this function is to update the delt about new start phase and new stop phase
        
        Parameters: Null
        ---------  
        Returns: True
        """
        #get start and stop point
        point_start  = self.SequencePoint[self.PointPhaseStart]
        point_stop  = self.SequencePoint[self.PointPhaseStop]
                        
        for xyz in range(self.Dimension):
                
            diff =   point_stop[xyz] - point_start[xyz]
              
            self.TnterpolationDelt[xyz] = diff/self.InterpolationNumber
                
        return True
                
    def getNewPoint(self):
        """this function is to get new value in the next interpolation point
        
        Parameters: Null
        ---------  
        Returns: self.pointnow, the new value of the new point 
        """
        #update movement tick
        self.ExecuteTick = self.ExecuteTick + 1
        if self.ExecuteTick > self.InterpolationNumber:
             self.ExecuteTick = 0
             self.updatePointPhase()
             self.updateInterpolationDelt()
        else:
            self.PointNow[0] = self.PointPrevious[0] + self.TnterpolationDelt[0]
            self.PointNow[1] = self.PointPrevious[1] + self.TnterpolationDelt[1]
            self.PointNow[2] = self.PointPrevious[2] + self.TnterpolationDelt[2]
            self.PointPrevious = self.PointNow

        return self.PointNow

class Movements:
    """this class used to set three interpolation of movement through the first class called sequenceInterpolation
       legs_location,speed and sttitude
    """
    def __init__(self,name,speed_enable = "SpeedEnable",attitude_enable = "AttitudeEnable",turn_enable = "TurnEnable",legs_enable = "LegsEnable"):

        self.MovementName =  name

        self.SpeedEnable = speed_enable
        self.AttitudeEnable = attitude_enable
        self.LegsEnable = legs_enable
        self.TurnEnable = turn_enable
        self.ExitToStand = False
        
        #legs transition speed and gait acceleration
        self.DeltLegsM = np.full((3,4), 0.0005)
        self.DeltSpeedM = np.full(3, 0.001)
        self.DeltAttitudeM = np.full(3, 1)
        self.DeltTurnM = np.full(3, 0)
        self.transTic = 70       #unit 1

        self.SpeedMovements = SequenceInterpolation('speed',2)
        self.AttitudeMovements = SequenceInterpolation('attitude',3)
        self.TurnMovements = SequenceInterpolation('turn',1)
        self.LegsMovements = []
        self.LegsMovements.append(SequenceInterpolation('leg1',3))
        self.LegsMovements.append(SequenceInterpolation('leg2',3))
        self.LegsMovements.append(SequenceInterpolation('leg3',3))
        self.LegsMovements.append(SequenceInterpolation('leg4',3))

        # init state value
        self.SpeedInit = [0,0,0]          # x, y speed
        self.AttitudeInit = [0,0,0]     # roll pitch yaw rate
        self.TurnInit = [0,0,0]         # yaw rate
        self.LegsLocationInit = [[0,0,0,0],[0,0,0,0],[0,0,0,0]] # x,y,z for 4 legs

        # output
        self.SpeedOutput = [0,0,0]          # x, y speed
        self.AttitudeOutput = [0,0,0]     # roll pitch yaw rate
        self.TurnOutput = [0,0,0]         # yaw rate
        self.LegsLocationOutput = [[0,0,0,0],[0,0,0,0],[0,0,0,0]] # x,y,z for 4 legs
        
    def setTransitionTic(self, tic):
        """ determin how many time steps it takes from current movement to the next one"""
        self.transTic = tic

    def setInterpolationNumber(self,number):
        """set interpolation number for three parts
        """    
        for leg in range(4):
            self.LegsMovements[leg].setInterpolationNumber(number)
        self.AttitudeMovements.setInterpolationNumber(number)          
        self.SpeedMovements.setInterpolationNumber(number)  
        self.TurnMovements.setInterpolationNumber(number)
        return True
        
    def setExitstate(self,state = "Continue"):
        """set exit state ,Stand is keep Stand after this move,Continue is keep this movement after this movement
        """
        if state == 'Stand':
            self.ExitToStand = True
        return True
        
    def setSpeedSequence(self,sequence,cycle_type = "single",cycle_index = 1):
        """set speed sequence

        Args:
            sequence (_type_): your design movement in speed part
            cycle_type (str, optional): three type :Forever, Multiple,single. Defaults to "single".
            cycle_index (int, optional): just can be set in Multiple type. Defaults to 1.
        """
        self.SpeedMovements.setSequencePoint(sequence)
        self.SpeedMovements.setCycleType(cycle_type,cycle_index)
        self.SpeedInit = sequence[0]


    def setAttitudeSequence(self,sequence,cycle_type = "single",cycle_index = 1):
        """set speed sequence

        Args:
            sequence (_type_): your design movement in attitude part
            cycle_type (str, optional): three type :Forever, Multiple,single. Defaults to "single".
            cycle_index (int, optional): just can be set in Multiple type. Defaults to 1.
        """
        self.AttitudeMovements.setSequencePoint(sequence)
        self.AttitudeMovements.setCycleType(cycle_type,cycle_index)
        self.AttitudeInit = sequence[0]

    def setTurnSequence(self,sequence,cycle_type = "single",cycle_index = 1):
        """set rotate_speed sequence

        Args:
            sequence (_type_): your design movement in turn part
            cycle_type (str, optional): three type :Forever, Multiple,single. Defaults to "single".
            cycle_index (int, optional): just can be set in Multiple type. Defaults to 1.
        """
        self.TurnMovements.setSequencePoint(sequence)
        self.TurnMovements.setCycleType(cycle_type,cycle_index)
        self.TurnInit = sequence[0]

    def setLegsSequence(self,sequence,cycle_type = "single",cycle_index = 1):
        """set four legs sequence

        Args:
            sequence (_type_): your design movement in legs_location part
            cycle_type (str, optional): three type :Forever, Multiple,single. Defaults to "single".
            cycle_index (int, optional): just can be set in Multiple type. Defaults to 1.
        """
        for leg in range(4):
            self.LegsMovements[leg].setSequencePoint(sequence[leg])
            self.LegsMovements[leg].setCycleType(cycle_type,cycle_index)
            
            # init location
            self.LegsLocationInit[0][leg] = sequence[leg][0][0]
            self.LegsLocationInit[1][leg] = sequence[leg][0][1]
            self.LegsLocationInit[2][leg] = sequence[leg][0][2]

    def setAllSequence(self,sequenceLeg,sequenceSpeed,sequenceAttitude):
        """for less code in danceSample
           you can through this function to set three part sequence interpolation when this parts type all is "single"
        """
        Movements.setLegsSequence(self,sequenceLeg)
        Movements.setSpeedSequence(self,sequenceSpeed)
        Movements.setAttitudeSequence(self,sequenceAttitude)            
        Movements.setTurnSequence(self,[[0,0,0]])

    def runLegsSequence(self):
        if self.LegsEnable == 'LegsEnable':
            for leg in range(4):
                leg_loaction = self.LegsMovements[leg].getNewPoint()
                for xyz in range(3):
                    self.LegsLocationOutput[xyz][leg] = leg_loaction[xyz]
    
    def runAttitudeSequence(self):
        if self.AttitudeEnable == 'AttitudeEnable':
            self.AttitudeOutput = self.AttitudeMovements.getNewPoint()

    def runSpeedSequence(self):
        if self.SpeedEnable == 'SpeedEnable':
            self.SpeedOutput = self.SpeedMovements.getNewPoint()
   
    def runTurnSequence(self):
        if self.TurnEnable == 'TurnEnable':
            self.TurnOutput = self.TurnMovements.getNewPoint()

    def getSpeedOutput(self, state = 'Normal'):
        """get every interpolation point value of speed
        """
        if state == 'Init':
            return self.SpeedInit
        else:
            return self.SpeedOutput

    def getAttitudeOutput(self, state = 'Normal'):
        """get every interpolation point value of attitude
        """
        if state == 'Init':
            return self.AttitudeInit
        else:
            return self.AttitudeOutput

    def getLegsLocationOutput(self, state = 'Normal'):
        """get every interpolation point value of legs_location
        """
        if state == 'Init':
            return self.LegsLocationInit
        else:
            return self.LegsLocationOutput

    def getTurnOutput(self, state = 'Normal'):
        """get every interpolation point value of rotate speed
        """
        if state == 'Init':
            return self.TurnInit
        else:
            return self.TurnOutput
    
    def getMovementName(self):
        """
        Returns:
            string : MovementName 
        """
        return self.MovementName
           
    def getCycleTicks(self):
        """the cycle ticks is the total number of interpolation in one movement 

        Returns:
        number
        """
        Speedticks = self.SpeedMovements.PhaseNumberMax * self.SpeedMovements.InterpolationNumber*self.SpeedMovements.ExecuteCounterMax
        Attitudeticks = self.AttitudeMovements.PhaseNumberMax * self.AttitudeMovements.InterpolationNumber*self.AttitudeMovements.ExecuteCounterMax
        Legsticks = self.LegsMovements[0].PhaseNumberMax * self.LegsMovements[0].InterpolationNumber*self.LegsMovements[0].ExecuteCounterMax 
        Turnticks = self.TurnMovements.PhaseNumberMax * self.TurnMovements.InterpolationNumber*self.TurnMovements.ExecuteCounterMax
         
        return max(Speedticks, Attitudeticks, Legsticks,Turnticks)
        
    def getPhaseNumberMax(self):
        """
        Returns:
        the phase number in one movement
        """
        return self.SpeedMovements.PhaseNumberMax, self.AttitudeMovements.PhaseNumberMax, self.LegsMovements[0].PhaseNumberMax,self.TurnMovements.PhaseNumberMax

class MovementScheme:
    """this class is to contact two movement,finally you can through this class to get a movementlib which have a series of movement 
    """
    def __init__(self,movements_lib):

        self.movements_lib = movements_lib

        self.movements_now = movements_lib[0]
        self.movements_pre = movements_lib[0]
        self.movement_now_name =  movements_lib[0].getMovementName()
        self.movement_now_number = 0
        self.transition = False
        
        self.ststus = 'Movement'    # 'Entry' 'Movement' 'Exit'
        self.entry_down = False
        self.entry_down1 = False
        self.entry_down2 = False
        self.entry_down3 = False
        self.exit_down = False
        self.exit_down1 = False
        self.exit_down2 = False
        self.exit_down3 = False
        self.tick = 0
        self.now_ticks = 0
        self.Legslocation_gradient_done_counter = 0
        self.Speed_gradient_done_counter = 0
        self.Turn_gradient_done_counter = 0
        self.Attitude_gradient_done_counter = 0
        self.DeltTurn = 40.0

        self.legs_location_pre = LocationStanding
        self.legs_location_now = LocationStanding
        self.speed_pre = SpeedStanding
        self.speed_now = SpeedStanding
        self.attitude_pre = AttitudeStanding
        self.attitude_now = AttitudeStanding
        
        self.legslocation_done_index = np.zeros((3,4))
        self.speed_done_index = [0,0,0]
        self.attitude_done_index = [0,0,0]
        self.turn_done_index = 0

        self.legslocation_gradient_done = False
        self.speed_gradient_done = False
        self.attitude_gradient_done = False
        self.turn_gradient_done = False

        self.attitude_pre = [0,0,0]
        self.attitude_now = [0,0,0]

        self.speed_pre = [0,0,0]
        self.speed_now = [0,0,0]
        
        self.turn_pre = [0,0,0]
        self.turn_now = [0,0,0]

        self.record_index_number = []
        
        # criterion that make sure delta of the movement transition are obtained at the begining of the transition and doesn't change later
        self.getAccCommand = True 

    def updateMovementType(self):
        """used to update movement ,caculate which movement should be move

        Returns:
            string : the new movement name
        """
        self.movements_pre = self.movements_lib[self.movement_now_number]
        self.movement_now_number = self.movement_now_number + 1
        if self.movement_now_number>= len(self.movements_lib):
            self.movement_now_number = self.movement_now_number - 1 
        self.entry_down = False
        self.entry_down1 = False
        self.entry_down2 = False
        self.entry_down3 = False
        self.exit_down = False
        self.exit_down1 = False
        self.exit_down2 = False
        self.exit_down3 = False
        self.transition = False
        self.movements_now = self.movements_lib[self.movement_now_number]

        return self.movements_now.getMovementName()

    def resetMovementNumber(self):
        """reset the movement ,let the action move from first movement

        """
        self.movements_pre = self.movements_lib[self.movement_now_number]
        self.movement_now_number = 0
        
        self.entry_down = False
        self.entry_down1 = False
        self.entry_down2 = False
        self.entry_down3 = False
        
        self.exit_down = False
        self.exit_down1 = False
        self.exit_down2 = False
        self.exit_down3 = False
        self.movements_now = self.movements_lib[0]

        return True
    
    def updateMovement(self,movement_type):
        """update movement, when the state is "entry",there will caculate the gradient about three parts(legs_location,speed,attitude) 
           when transition between two movement

        Args:
            movement_type (string): the name of the movement
        """
        # movement state transition
        if (movement_type != self.movement_now_name):
            self.ststus = 'Exit'
        elif(self.entry_down and self.entry_down1 and self.entry_down2 and self.entry_down3 ):
            self.ststus = 'Movement'
        elif(self.exit_down and self.exit_down1 and self.exit_down2 and self.exit_down3 ):
            self.ststus = 'Entry'
        
        
        self.movement_now_name = movement_type
        self.now_ticks = self.movements_now.getCycleTicks()

        # update system tick
        
        # movement execute
        if self.ststus == 'Entry':
        #this part is superimposed model
#             if self.movements_now.getMovementName() == 'stop':
#             
#                 self.record_index_number = []
#                 location_ready = self.movements_now.getLegsLocationOutput('Init')
#                 speed_ready = self.movements_now.getSpeedOutput('Init')
#                 attitude_ready = self.movements_now.getAttitudeOutput('Init')
#                 
#             else:
#
#                 location_ready = np.array(self.movements_now.getLegsLocationOutput('Init'))
#                 speed_ready = np.array(self.movements_now.getSpeedOutput('Init'))
#                 attitude_ready = np.array(self.movements_now.getAttitudeOutput('Init'))
#                 
#                 if (self.movement_now_number - 1) not in self.record_index_number: 
#                     self.record_index_number.append(self.movement_now_number - 1)
#                 print("movement_now_number",self.movement_now_number)
#                 for number_index in range(len(self.record_index_number)):
#                     print("record_index_number",self.record_index_number)
#                     speed_ready = np.array(speed_ready) + np.array(self.movements_lib[self.record_index_number[number_index]].getSpeedOutput('Init'))
#                     attitude_ready = np.array(attitude_ready) + np.array(self.movements_lib[self.record_index_number[number_index]].getAttitudeOutput('Init'))

             location_ready = np.array(self.movements_now.getLegsLocationOutput('Init')) #a matrix with 3 rows and 4 columns
             speed_ready = np.array(self.movements_now.getSpeedOutput('Init'))
             attitude_ready = np.array(self.movements_now.getAttitudeOutput('Init')) 
             turn_ready = np.array(self.movements_now.getTurnOutput('Init'))
             
             if self.getAccCommand == True:
                 self.movements_now.DeltLegsM = location_ready - self.legs_location_pre
                 self.movements_now.DeltLegsM  = self.movements_now.DeltLegsM.astype(np.float64)
                 self.movements_now.DeltLegsM /= self.movements_now.transTic
                 
                 self.movements_now.DeltSpeedM = speed_ready - self.speed_pre
                 self.movements_now.DeltSpeedM = self.movements_now.DeltSpeedM.astype(np.float64)
                 self.movements_now.DeltSpeedM /= self.movements_now.transTic
                 
                 self.movements_now.DeltAttitudeM = attitude_ready - self.attitude_pre
                 self.movements_now.DeltAttitudeM = self.movements_now.DeltAttitudeM.astype(np.float64)
                 self.movements_now.DeltAttitudeM /= self.movements_now.transTic
                 
                 self.movements_now.DeltTurnM = turn_ready - self.turn_pre
                 self.movements_now.DeltTurnM = self.movements_now.DeltTurnM.astype(np.float64)
                 self.movements_now.DeltTurnM /= self.movements_now.transTic
                 self.getAccCommand = False
             
             
             self.legs_location_now, self.entry_down = self.updateMovementLegsLocationGradient(self.legs_location_pre,location_ready)
             self.legs_location_pre = self.legs_location_now
             
             self.speed_now, self.entry_down1 = self.updateMovementSpeedGradient(self.speed_pre,speed_ready)
             self.speed_pre = self.speed_now
             
             self.attitude_now, self.entry_down2 = self.updateMovementAttitudeGradient(self.attitude_pre,attitude_ready)
             self.attitude_pre = self.attitude_now
             
             self.turn_now, self.entry_down3 = self.updateMovementTurnGradient(self.turn_pre,turn_ready)
             self.turn_pre = self.turn_now
             
             if self.entry_down == True and self.entry_down1 == True and self.entry_down2 == True and self.entry_down3 == True :
             
                 self.Legslocation_gradient_done_counter = 0
                 self.speed_gradient_done_counter = 0
                 self.attitude_gradient_done_counter = 0
                 self.turn_gradient_done_counter = 0
                 
                 self.exit_down = False
                 self.exit_down1 = False
                 self.exit_down2 = False
                 self.exit_down3 = False
                 self.legslocation_gradient_done = False
                 self.speed_gradient_done = False
                 self.attitude_gradient_done = False
                 self.turn_gradient_done = False

                 self.legslocation_done_index = np.zeros((3,4))
                 self.speed_done_index = [0,0]
                 self.turn_done_index = 0
                 self.attitude_done_index = [0,0,0]
                 self.getAccCommand = True

        if self.ststus == 'Exit':
             if self.movements_pre.ExitToStand == True:
                  #update the legslocation
                  self.legs_location_now, self.exit_down=self.updateMovementLegsLocationGradient(self.legs_location_pre,LocationStanding)
                  self.legs_location_pre = self.legs_location_now
                  
                  #update the speed
                  self.speed_now, self.exit_down1=self.updateMovementSpeedGradient(self.speed_pre,SpeedStanding)
                  self.legs_location_pre = self.legs_location_now
                  
                  #update the attitude
                  self.attitude_now, self.exit_down2=self.updateMovementAttitudeGradient(self.attitude_pre,AttitudeStanding)
                  self.attitude_pre = self.attitude_now
                  
                  #update the rotate speed
                  self.turn_now, self.exit_down3=self.updateMovementAttitudeGradient(self.turn_pre,0)
                  self.turn_pre = self.turn_now
                  
                  if self.exit_down == True and self.exit_down1 == True and self.exit_down2 == True and self.exit_down3 :
                  
                      self.Legslocation_gradient_done_counter = 0
                      self.Speed_gradient_done_counter = 0
                      self.Attitude_gradient_done_counter = 0
                      self.Turn_gradient_done_counter = 0

                      self.legslocation_done_index = np.zeros((3,4))
                      self.speed_done_index = [0,0]
                      self.attitude_done_index = [0,0,0]
                      self.turn_done_index = 0

                      self.entry_down = False
                      self.entry_down1 = False
                      self.entry_down2 = False
                      self.entry_down3 = False
                      self.legslocation_gradient_done = False
                      self.speed_gradient_done = False
                      self.attitude_gradient_done = False
                      self.turn_gradient_done = False

             else:
                  self.legs_location_now = self.legs_location_pre
                  self.speed_now = self.speed_pre
                  self.attitude_now = self.attitude_pre
                  self.turn_now = self.turn_pre
                  self.exit_down = True
                  self.exit_down1 = True
                  self.exit_down2 = True
                  self.exit_down3 = True
                  self.entry_down = False
                  self.entry_down1 = False
                  self.entry_down2 = False
                  self.entry_down3 = False
                  self.legslocation_gradient_done = False
                  self.speed_gradient_done = False
                  self.attitude_gradient_done = False
                  self.turn_gradient_done = False
                  self.Legslocation_gradient_done_counter = 0
                  self.Speed_gradient_done_counter = 0
                  self.Attitude_gradient_done_counter = 0
                  self.Turn_gradient_done_counter = 0


        elif self.ststus == 'Movement':
             Nos, Noa, Nol, Not = self.movements_now.getPhaseNumberMax()
             if Nol > 1 :
                 self.updateLegsScheme()
             if Noa > 1 :
                 self.updateAttitudeScheme()
             if Nos > 1 :
                 self.updateSpeedScheme()
             if Not > 1 :
                 self.updateTurnScheme()
             
             self.legs_location_pre = self.legs_location_now
             self.speed_pre = self.speed_now 
             self.attitude_pre = self.attitude_now
             self.turn_pre = self.turn_now

             self.tick += 1
             if self.tick >= self.now_ticks  and self.movement_now_number < len(self.movements_lib) - 1:
                 self.transition = True
                 self.tick = 0

        return self.legs_location_now,self.speed_now,self.attitude_now, self.turn_now

    def updateMovementLegsLocationGradient(self,location_now,location_target):
        """uodate gradient legs_locaiton between two movement

        Args:
            location_now (array 3*4): now legs_location
            location_target (arrar 3*4): target legs_location

        Returns:
        location_gradient: new legs_locaiton in transition
        self.legslocation_gradient_done: the state about legs_locaiton transition
        """
        loaction_gradient = location_now

        #legs gradient
        for xyz_index in range(3):
            for leg_index in range(4):

                diff = location_target[xyz_index][leg_index] - location_now[xyz_index][leg_index]
                if abs(diff) > abs(self.movements_now.DeltLegsM[xyz_index][leg_index]):
                    loaction_gradient[xyz_index][leg_index] = location_now[xyz_index][leg_index] + self.movements_now.DeltLegsM[xyz_index][leg_index]               
                else :
                    loaction_gradient[xyz_index][leg_index] = location_target[xyz_index][leg_index]
                    if self.legslocation_done_index[xyz_index][leg_index] != 1:
                        self.Legslocation_gradient_done_counter += 1
                        self.legslocation_done_index[xyz_index][leg_index] = 1
                    
        if self.Legslocation_gradient_done_counter == 12:
            
            self.legslocation_gradient_done = True

        return loaction_gradient,self.legslocation_gradient_done
        
    def updateMovementSpeedGradient(self,speed_now,speed_target):
        """uodate gradient speed between two movement

        Args:
            speed_now (array 1*3): now speed
            speed_target (arrar 1*3): target speed

        Returns:
        speed_gradient: new speed in transition
        self.speed_gradient_done: the state about speed transition
        """
        speed_gradient = speed_now
        gradient_done = False

        #speed gradient
        for xy_index in range(2):
                diff = speed_target[xy_index] - speed_now[xy_index]
                if abs(diff) > abs(self.movements_now.DeltSpeedM[xy_index]):
                    speed_gradient[xy_index] = speed_now[xy_index] + self.movements_now.DeltSpeedM[xy_index]               
                else :
                    speed_gradient[xy_index] = speed_target[xy_index]
                    if self.speed_done_index[xy_index] != 1 :
                        self.speed_done_index[xy_index] = 1
                        self.Speed_gradient_done_counter += 1
                    
        if self.Speed_gradient_done_counter == 2:
            
            self.speed_gradient_done = True

        return speed_gradient,self.speed_gradient_done

    def updateMovementAttitudeGradient(self,attitude_now,attitude_target):
        """uodate gradient attitude between two movement

        Args:
            attitude_now (array 1*3): now attitude
            attitude_target (arrar 1*3): target attitude

        Returns:
        attitude_gradient: new attitude in transition
        self.attitude_gradient_done: the state about attitude transition
        """
        attitude_gradient = attitude_now
        #Attitude gradient
        for rpy_index in range(3):
                diff = attitude_target[rpy_index] - attitude_now[rpy_index]
                if abs(diff) > abs(self.movements_now.DeltAttitudeM[rpy_index]):
                    attitude_gradient[rpy_index] = attitude_now[rpy_index] + self.movements_now.DeltAttitudeM[rpy_index]
                else :
                    attitude_gradient[rpy_index] = attitude_target[rpy_index]
                    if self.attitude_done_index[rpy_index] != 1 :
                        self.Attitude_gradient_done_counter += 1
                        self.attitude_done_index[rpy_index] = 1
                    
        if self.Attitude_gradient_done_counter == 3:
            
            self.attitude_gradient_done = True

        return attitude_gradient,self.attitude_gradient_done
     
    def updateMovementTurnGradient(self,turn_now,turn_target):
        """update gradient rotate speed between two movement

        Args:
            turn_now (array 1*3): now rotate speed
            turn_target (arrar 1*3): target rotate speed

        Returns:
        turn_gradient: new turning in transition
        self.turn_gradient_done: the state about turning transition
        """
        turn_gradient = turn_now
        diff = turn_target[0] - turn_now[0]
        if abs(diff) > abs(self.movements_now.DeltTurnM[0]):
            turn_gradient[0] = turn_now[0] + self.movements_now.DeltTurnM[0]
        else :
            turn_gradient[0] = turn_target[0]
            if self.Turn_gradient_done_counter != 1:
                self.Turn_gradient_done_counter += 1
                
        if self.Turn_gradient_done_counter == 1:
            self.turn_gradient_done = True
            
        return turn_gradient,self.turn_gradient_done 
            
    def updateLegsScheme(self):
        self.movements_now.runLegsSequence()
        self.legs_location_now = self.movements_now.getLegsLocationOutput('normal')
         
    def updateAttitudeScheme(self):
        self.movements_now.runAttitudeSequence()
        self.attitude_now = self.movements_now.getAttitudeOutput('normal')
         
    def updateSpeedScheme(self):
        self.movements_now.runSpeedSequence()
        self.speed_now = self.movements_now.getSpeedOutput('normal')
        
    def updateTurnScheme(self):
        self.movements_now.runTurnSequence()
        self.turn_now = self.movements_now.getTurnOutput('normal')
        
    def runMovementScheme(self):
        """run the movement in movementlib
        """
        # update movement
        movement_name = ' '
        if self.transition == True:
            movement_name = self.updateMovementType()
            
        self.updateMovement(movement_name) 

    def getMovemenSpeed(self):
        """get now speed
        """
        speed_now = np.zeros(2,dtype=np.float64)
        for xyz in range(2):
            speed_now[xyz] = self.speed_now[xyz]
       
        return speed_now

    def getMovemenLegsLocation(self):
        """get now legs_location
        """
        return self.legs_location_now 

    def getMovemenAttitude(self):
        """get now attitude
        """
        return ture

    def getMovemenTurn(self):
        """get now turn
        """
        turn_now_rad = [0.0,0.0,0.0]
        turn_now_rad[0] = self.turn_now[0]/57.3

        return turn_now_rad[0]
