import pybullet as p
import pybullet_data
import os
import numpy as np
import sys
import pandas as pd
sys.path.insert(1, 'utils/')
from simulation import sim
from Robot import Robot
from thing import Thing
from pointCloud import getPointCloud
from IPython import embed 

class robotSimulator():
    def __init__(self):
        self.sim = sim()

        self.plane = Thing("plane")

        self.pose = []
        #Create robot object with given arm and gripper
        self.robot = Robot(self.sim.robotParams, self.sim.motionParams["grip_path"])
        self.initButtonVals()
        self.loop()

    def loop(self):
        while(True):
            self.readResetButton()
            self.readToTargetButton()
            self.readGripButton()
            self.readOpenGripperButton()
            self.readTableButton()
            self.readObjectButton()
            self.readPlanPathButton()
            self.readFollowPathButton()
            self.readCameraButton()
            self.readSavePoseButton()
            self.readFollowedSavedPathButton()
            self.readPointCloudButton()
            self.readJointControlButton()
            self.readmultiGraspButton()

            if self.sim.robotParams["camera"]:
                self.robot.simStep()

    def initButtonVals(self):
        self.startButtonVal = 2.0
        self.gripButtonVal = 2.0
        self.openGripperButtonVal = 2.0
        self.resetButtonVal = 2.0
        self.tableButtonVal = 2.0
        self.planButtonVal = 2.0
        self.pathButtonVal = 2.0
        self.cameraButtonVal = 2.0
        self.objButtonVal = 2.0
        self.savePoseButtonVal = 2.0
        self.followSavedPathButtonVal = 2.0
        self.pointCloudButtonVal=2.0
        self.jointControlButtonVal = 2.0
        self.prevVal = 0
        self.multiGraspButtonVal = 2.0

    def readResetButton(self):
        if p.readUserDebugParameter(self.sim.resetButton) >= self.resetButtonVal:
            self.robot.setJointPosition(np.zeros((self.robot.numArmJoints+self.robot.numGripperJoints)))
            if self.tableButtonVal >2.0:
                p.resetBasePositionAndOrientation(self.robot.robot_model,[0,0,self.table.height], self.robot.start_orientation)
            else:
                p.resetBasePositionAndOrientation(self.robot.robot_model, self.robot.start_pos, self.robot.start_orientation)
            self.resetButtonVal = p.readUserDebugParameter(self.sim.resetButton) + 1.0

    def readToTargetButton(self):
        if p.readUserDebugParameter(self.sim.startButton) >= self.startButtonVal:
            #Read data
            data = self.sim.motionParams["pose_target"]

            data = self.robot.createDQObject(data)
        
            self.robot.moveArmToEETarget(data,0.6)
            self.startButtonVal = p.readUserDebugParameter(self.sim.startButton) + 1.0
            
    def readGripButton(self):
        if p.readUserDebugParameter(self.sim.gripButton) >= self.gripButtonVal:
            self.robot.gripper.grip()
            self.gripButtonVal = p.readUserDebugParameter(self.sim.gripButton) + 1.0

    def readOpenGripperButton(self): 
        if p.readUserDebugParameter(self.sim.openGripperButton) >= self.openGripperButtonVal:
            self.robot.gripper.openGripper()
            self.openGripperButtonVal = p.readUserDebugParameter(self.sim.openGripperButton) + 1.0

    #调用thing.py里的thing类，创建桌子
    def readTableButton(self):
        if p.readUserDebugParameter(self.sim.tableButton) >= self.tableButtonVal:
            self.table = Thing("table")
            p.resetBasePositionAndOrientation(self.robot.robot_model, [0,0,self.table.height], self.robot.start_orientation)
            self.tableButtonVal = p.readUserDebugParameter(self.sim.tableButton) + 1.0
    #调用thing.py里的thing类，创建物体
    def readObjectButton(self):
        if p.readUserDebugParameter(self.sim.objButton) >= self.objButtonVal:
            #把物体生成在桌子上
            if self.tableButtonVal > 2.0:
                self.container = Thing(self.sim.simParams["object_name"],onObject=self.table)
            #把物体生成在 self.sim.simParams["object_position"]
            else: 
                self.container = Thing(self.sim.simParams["object_name"],self.sim.simParams["object_position"])
            self.objButtonVal = p.readUserDebugParameter(self.sim.objButton) + 1.0

    def readPlanPathButton(self):
        if p.readUserDebugParameter(self.sim.planButton) >= self.planButtonVal:
            #TODO: add path planning
            print("[INFO]: not yet implemented")
            self.planButtonVal = p.readUserDebugParameter(self.sim.planButton) + 1.0

    def readFollowPathButton(self):
        if p.readUserDebugParameter(self.sim.pathButton) >= self.pathButtonVal:
            #TODO: add follow path
            print("[INFO]: not yet implemented")
            self.pathButtonVal = p.readUserDebugParameter(self.sim.pathButton) + 1.0

    def readCameraButton(self):
        if p.readUserDebugParameter(self.sim.cameraButton) >= self.cameraButtonVal:
            if hasattr(self,"container"):
                self.sim.getPicture(self.container.getPos())
            else:
                self.sim.getPicture()

            self.cameraButtonVal = p.readUserDebugParameter(self.sim.cameraButton) + 1.0

    def readSavePoseButton(self):
        #保存EE的Position和Orientation
        if p.readUserDebugParameter(self.sim.savePoseButton) >= self.savePoseButtonVal:
            currPose = self.robot.getPose()
            self.pose.append([currPose[0],currPose[1]])
            print("Saved EE World Position : ",currPose[0])
            print("Saved EE World Orientation (Quaternion) : ",currPose[1])
            self.savePoseButtonVal = p.readUserDebugParameter(self.sim.savePoseButton) + 1.0
    
    def readFollowedSavedPathButton(self):
        #移动倒最后一个保存位置
        if p.readUserDebugParameter(self.sim.followSavedPathButton) >= self.followSavedPathButtonVal:
            if len(self.pose) > 0:
                for i in range(len(self.pose)):
                    self.robot.followPath(self.pose[i][0])
            else:
                print("No position saved")
        self.followSavedPathButtonVal = p.readUserDebugParameter(self.sim.followSavedPathButton) + 1.0

            
    def readPointCloudButton(self):
        if p.readUserDebugParameter(self.sim.pointCloudButton) >= self.pointCloudButtonVal:
            if hasattr(self,"container"):
                self.sim.getPointCloud(self.container.getPos(), self.robot.getPandO())
            else:
                self.sim.getPointCloud()
            self.pointCloudButtonVal = p.readUserDebugParameter(self.sim.pointCloudButton) + 1.0

    def readJointControlButton(self):
        if p.readUserDebugParameter(self.sim.jointControlButton) == self.jointControlButtonVal:
            self.sim.addjointControl(self.robot.numArmJoints,self.robot.getJointPosition())
            self.jointControlButtonVal = 1
            self.prevVal = self.sim.getJointControlVal()
        
        if self.jointControlButtonVal == 1:
            targetPos = self.sim.getJointControlVal()
            if self.prevVal != targetPos:
                self.robot.setJointPosition(targetPos)
                self.prevVal = self.sim.getJointControlVal()
    
    def readmultiGraspButton(self):
        if p.readUserDebugParameter(self.sim.multiGraspButton) >= self.multiGraspButtonVal:
            self.robot.multiGraspSimulation()
            self.multiGraspButtonVal = p.readUserDebugParameter(self.sim.multiGraspButton) + 1.0

if __name__ == "__main__":
    main = robotSimulator()

    