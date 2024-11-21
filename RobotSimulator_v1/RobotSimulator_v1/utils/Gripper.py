import numpy as np
import scipy.linalg as LA
import time
from time import sleep
import pybullet as p
import dqrobotics 
from dqrobotics import robot_modeling
import pandas as pd

class gripper:
    """ The parent gripper class """

    def __init__(self,robot):
        self.robot = robot
        self.robot_model = robot.robot_model
        self.jointOffset = robot.numArmJoints
        self.numJoints = robot.numGripperJoints
        self.open = False
        
        #Read grip data
        jointNames = []
        for j in range(self.numJoints):
            jointNames.append('j'+str(j))
        data = pd.read_table(self.robot.gripPath,sep=",",names=jointNames)
        dataArr = []
        for joint in jointNames:
            dataArr.append(np.array(data[joint].values))
            
        self.gripData = np.stack(dataArr,axis =-1)

        
    def openGripper(self):
        pass
    
    def grip(self):
        """ Grip according to a predefined motion """    
        length = self.gripData.shape[1]
       
        for pos in self.gripData:
            length = len(pos)
            for i in range(length):
                p.setJointMotorControl2(self.robot_model, i + self.jointOffset, p.POSITION_CONTROL, targetPosition=pos[i],maxVelocity=2,force=4)
            self.robot.simStep()
        self.open = False

    def getGraspForceData(self):
        """Returns the current force data"""

        forcegain = np.zeros(self.numJoints)
        for i in range(self.numJoints):
            forcegain[i] = p.getJointState(self.robot_model, i + self.jointOffset)[3]
        
        return forcegain

    def getJointPosition(self):
        joints = []
        for i in range(0, self.numJoints):
            joints.append(p.getJointState(self.robot_model, i + self.jointOffset)[0])

        return joints


class threeFingers(gripper):

    """KUKA iiwa 3 finger gripper class"""
    
    def __init__(self,robot):
        gripper.__init__(self,robot)
        p.changeDynamics(self.robot_model, 2 + self.jointOffset, lateralFriction=2,
                         rollingFriction=0,frictionAnchor=1)
        p.changeDynamics(self.robot_model, 8 + self.jointOffset, lateralFriction=2,
                         rollingFriction=0,frictionAnchor=1)
        p.changeDynamics(self.robot_model, 5 + self.jointOffset, lateralFriction=2,
                         rollingFriction=0,frictionAnchor=1)

    def openGripper(self):
        closed = True
        iteration = 0
        while(closed and not self.open):
            joints = self.getJointPosition()
            closed = False
            for k in range(0,self.numJoints):
                #lower finger joints
                if k==2 or k==5 or k==8:
                    goal = -0.9
                    if joints[k] >= goal:    
                        p.setJointMotorControl2(self.robot_model, k + self.jointOffset, p.POSITION_CONTROL,targetPosition=joints[k] - 0.05, maxVelocity=2,force=5)   
                        closed = True
                #Upper finger joints             
                elif k==6 or k==3 or k==9:
                    goal = 0.5
                    if joints[k] <= goal:
                        p.setJointMotorControl2(self.robot_model, k + self.jointOffset, p.POSITION_CONTROL,targetPosition=joints[k] + 0.05,maxVelocity=2,force=5)
                        closed = True
                #Base finger joints
                elif k==1 or k==4:
                    pos = 0.5
                    if joints[k] <= pos:
                        p.setJointMotorControl2(self.robot_model, k + self.jointOffset, p.POSITION_CONTROL,targetPosition=joints[k] + 0.05,maxVelocity=2,force=5)
                        closed = True
            iteration += 1
            if iteration > 10000:
                break
            self.robot.simStep()
        self.open = True

class RG6(gripper):

    def __init__(self,robot):
        gripper.__init__(self,robot)

    def openGripper(self):
        pass


class yumi(gripper):

    def __init__(self,robot):
        gripper.__init__(self,robot)

    def openGripper(self):
        pass


class shadowHand(gripper):

    def __init__(self,robot):
        gripper.__init__(self,robot)

    def openGripper(self): #29 joints all should be 0 when open
        closed = True
        iteration = 0
        while (closed):
            joints = self.getJointPosition()
            closed = False
            for k in range(29):
                if joints[k] >= 0.05:
                    p.setJointMotorControl2(self.robot_model, k + self.jointOffset, p.POSITION_CONTROL,targetPosition=joints[k] - (joints[k]/abs(joints[k]))*0.01,maxVelocity=2,force=5)
                    closed = True
            iteration += 1
            if iteration > 50000:
                break
            