import numpy as np
import scipy.linalg as LA
import time
from time import sleep
import pybullet as p
import dqrobotics 
from dqrobotics import robot_modeling
from dqrobotics.robot_modeling import DQ_Kinematics
from dqrobotics import DQ
from Gripper import RG6, threeFingers, yumi, shadowHand
import math
from math import pi
import os
import pandas as pd
import pybullet_data
#import pybullet_planning

class Robot:
    """
    Input: Dict{arm:"name", gripper:"name", robot_start_pos:[x,y,z], robot_start_orientation_euler:[x,y,z]}

    Creates a robot object based on wanted robot. 
    Possible to move robot arm as wanted
    """

    def __init__(self, params, gripPath):
        
        """Initiates constants and a SerialManiulator object based on choosen robot"""

        self.arm = params["arm"]
        self.gripperName = params["gripper"]
        self.start_pos = params["robot_start_pos"]
        self.start_orientation = p.getQuaternionFromEuler(params["robot_start_orientation_euler"])
        self.gripPath = gripPath
        #Initiate choosen robot
        """
        TODO: should make it possible to change between different gripper/arm combinations when urdf files are available.
        """
        
        if self.arm == "ur10":
            self.DH = np.load("Robots/UR/DH/ur_dh.npy")
            # self.serialManipulator = robot_modeling.DQ_SerialManipulator(self.DH, 'standard')
            self.serialManipulator = robot_modeling.DQ_SerialManipulatorDH(np.vstack([self.DH, [1,1,1,1,1,1]]))
            
            if self.gripperName == "threeFingers":
                self.robot_model = p.loadURDF("Robots/UR/ur10/ur10_3fing.urdf", 
                    self.start_pos, self.start_orientation, useFixedBase=1)
            elif self.gripperName == "RG6":
                self.robot_model = p.loadURDF("Robots/UR/ur10/ur10_rg6.urdf", self.start_pos, 
                    self.start_orientation, useFixedBase=1)
            elif self.gripperName == "shadowHand":
                self.robot_model = p.loadURDF("Robots/UR/ur10/ur10_sh.urdf", self.start_pos, 
                    self.start_orientation, useFixedBase=1) 

        elif self.arm == "Kuka":
            self.DH = np.load("Robots/KUKA/DH/kuka_dh.npy")
            #self.serialManipulator = robot_modeling.DQ_SerialManipulator(self.DH, 'standard')
            self.serialManipulator = robot_modeling.DQ_SerialManipulatorDH(np.vstack([self.DH, [1,1,1,1,1,1,1]]))
            
            
            if self.gripperName == "threeFingers":
                self.robot_model = p.loadURDF("Robots/KUKA/kuka_3fing.urdf", self.start_pos, 
                    self.start_orientation, useFixedBase=1)
            elif self.gripperName == "RG6":
                self.robot_model = p.loadURDF("Robots/KUKA/kuka_RG6.urdf", self.start_pos, 
                    self.start_orientation, useFixedBase=1)
            elif self.gripperName == "shadowHand":
                self.robot_model = p.loadURDF("Robots/KUKA/kuka_sh.urdf", self.start_pos, 
                    self.start_orientation, useFixedBase=1)
            elif self.gripperName == "PR2":
                self.robot_model = p.loadURDF("pr2_gripper.urdf", self.start_pos, 
                    self.start_orientation, useFixedBase=1)
                
        elif self.arm == "Yumi":
            self.DH = np.load("Robots/yumi_description/DH/yumi_dh.npy")
            #self.serialManipulator = robot_modeling.DQ_SerialManipulator(self.DH, 'standard')
            self.serialManipulator = robot_modeling.DQ_SerialManipulatorDH(np.vstack([self.DH, [1,1,1,1,1,1,1]]))
            
            self.robot_model = p.loadURDF("Robots/yumi_description/urdf/yumi.urdf", self.start_pos, self.start_orientation, useFixedBase=1)

        else:
            print("[ERROR]: No robot available by that name, edit parameter file")
            quit
        
        #---------------------------有用的参数----------------------
        self.numArmJoints = self.DH.shape[1]
        #self.numGripperJoints = p.getNumJoints(self.robot_model)-self.numArmJoints
        #Find GripperJoints
        num_joints = p.getNumJoints(self.robot_model)
        movable_joints = []
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_model, i)
            joint_type = joint_info[2]  # 第三个返回值是关节类型
            if joint_type != p.JOINT_FIXED:  # 过滤固定关节
                movable_joints.append(i)
        self.numGripperJoints = len(movable_joints) - self.numArmJoints
        self.EEindex=int(self.numArmJoints)+1
        self.total_joints=self.numArmJoints+self.numGripperJoints
        

        #setBaseFrame
        robotPos, robotOrientation = p.getBasePositionAndOrientation(self.robot_model) 
        robotJointInfo = p.getJointInfo(self.robot_model,0)
        basekuka = DQ(np.array([robotOrientation[3], robotOrientation[0], robotOrientation[1], 
                                robotOrientation[2], 0, 0, 0, 0 ]))
        basekuka = basekuka * (basekuka.norm().inv())
        basekuka = basekuka + DQ([0, 0, 0, 0, 0, 0.5*(robotPos[0]+robotJointInfo[14][0]), 
                              0.5*(robotPos[1]+robotJointInfo[14][1]), 
                              0.5*(robotPos[2]+robotJointInfo[14][2])])*basekuka
        
        #robot_modeling.DQ_Kinematics.set_reference_frame(basekuka)
        self.serialManipulator.set_reference_frame(basekuka)

        if self.gripperName == "RG6":
            self.gripper = RG6(self)
        elif self.gripperName == "threeFingers":
            self.gripper = threeFingers(self)
        elif self.gripperName == "yumi":
            self.gripper = yumi(self)
        elif self.gripperName == "shadowHand":
            self.gripper = shadowHand(self)

        #Constants
        self.K = params["K"]
        self.enableCamera = params["camera"]
        self.homePos = self.getJointPosition()
        if self.enableCamera:
            self.cameraPos = params["camera_pos"]
            self.camera()


    def setJointPosition(self, targetJointPos):

        """ Set robot to a fixed position """

        for j in range(self.numArmJoints):
            p.resetJointState(self.robot_model, j, targetJointPos[j])      

    def getJointPosition(self):
        """
        Return joint position for each joint as a list
        """
        num_joints = p.getNumJoints(self.robot_model)
        print(f"Total joints: {num_joints}")
        return [p.getJointState(self.robot_model,j)[0] for j in range(self.numArmJoints)]
    
    def getPose(self):
        """ return pose"""
        #theta = self.getJointPosition()
        #pose = self.serialManipulator.fkm(theta)
        
        #----获取EE（index:7+1）的Position和Orientation----
        EE_state = p.getLinkState(self.robot_model, self.EEindex, computeForwardKinematics=True)
        EE_WorldPosition = EE_state[0]
        EE_WorldOrientation= EE_state[1]
        EE_pose = [EE_WorldPosition,EE_WorldOrientation]
        return EE_pose

    def getJointVelocity(self):
        """
        Return joint velocity for each joint as a list
        """
        return [p.getJointState(self.robot_model,j)[1] for j in range(self.numArmJoints)]
    
    def moveArmToEETarget(self, EE_target, epsilon):
        print("-----------------")
        print(self.numGripperJoints)
        target_joint_positions = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_model,# 机器人模型的 ID
            endEffectorLinkIndex=self.EEindex, # 末端执行器的链接索引
            targetPosition=EE_target,
            maxNumIterations=5000,# 目标位置 (x, y, z)，世界坐标系
            residualThreshold=1e-8
            )
        print("-----------------")
        print(len(target_joint_positions))
        #PD控制 7个手臂关节
        for i in range(self.numArmJoints):
            p.setJointMotorControl2(self.robot_model,jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=target_joint_positions[i],
                                    force=500,
                                    maxVelocity=3,
                                    positionGain=0.1,
                                    velocityGain=0.8)
        #p.stepSimulation()
        time.sleep(1)
            
        #顺着路径移动
        # current_joint_positions = [p.getJointState(self.robot_model, i)[0] for i in range(len(target_joint_positions))]
        # steps = 100
        # trajectory = np.linspace(current_joint_positions, target_joint_positions, steps)
        # print(trajectory.shape)
        # for step in trajectory:
        #     for i, joint_position in enumerate(step):
        #         p.setJointMotorControl2(self.robot_model, i, p.POSITION_CONTROL, joint_position)
        #     p.stepSimulation()
        #     time.sleep(0.01) 
            
        #直接瞬移过去
        # for i in range(self.numArmJoints):
        #     p.resetJointState(self.robot_model, i, target_joint_positions[i])

    def followPath(self, poses):
        """
        input: unit dual quaternions
        """
        print("Moving to pose", poses)
        self.moveArmToEETarget(poses,0.4)
        print("Reached pose")

    #def homePos(self)


    def pause(self, s):
        """
        run simulation with any movements for s seconds
        """

        t_end = time.time() + s
        while time.time() < t_end:
            p.stepSimulation()

    def camera(self):
        
        """ 
        Show camera image at given position 

        TODO: Edit so that the camera positon is changable
        """

        fov, aspect, nearplane, farplane = 60, 1.0, 0.01, 100
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
        if self.arm is not None:
            linkPos = self.numArmJoints-1
        else:
            linkPos = self.numGripperJoints-1

        com_p, com_o, _, _, _, _ = p.getLinkState(self.robot_model, linkPos, computeForwardKinematics=True)
        rot_matrix = p.getMatrixFromQuaternion(com_o)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        com_o_Eul=p.getEulerFromQuaternion(com_o)
        #print(com_o_Eul)
        normtemp=np.linalg.norm(com_o_Eul)
        
        #quickfix
        com_p1= com_p[0]-(0.2/normtemp)*com_o_Eul[0]
        com_p2= com_p[1]-(0.2/normtemp)*com_o_Eul[1]
        com_p3= com_p[2]-(0.2/normtemp)*com_o_Eul[2]
        com_p=(com_p1,com_p2,com_p3)
        init_camera_vector = (0, 0, 1) # z-axis
        init_up_vector = (0, 1, 0) # y-axis

        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(com_p, com_p + 0.5 * camera_vector, up_vector)
        img = p.getCameraImage(200, 200, view_matrix, projection_matrix)
        
        return img
    def getPandO(self):
        if self.arm is not None:
            linkPos = self.numArmJoints-1
        else:
            linkPos = self.numGripperJoints-1
        com_p, com_o, _, _, _, _ = p.getLinkState(self.robot_model, linkPos, computeForwardKinematics=True)
        
        com_o_Eul=p.getEulerFromQuaternion(com_o)
        #print(com_o_Eul)
        normtemp=np.linalg.norm(com_o_Eul)
        
       #quickfix 
        com_p1= com_p[0]-(0.2/normtemp)*com_o_Eul[0]
        com_p2= com_p[1]-(0.2/normtemp)*com_o_Eul[1]
        com_p3= com_p[2]-(0.2/normtemp)*com_o_Eul[2]
        com_p=(com_p1,com_p2,com_p3)
        return com_p


    def simStep(self):
        """
        Simulation step, should be used whenever robot is moved, simStep is set automatically if visuals is on
        """
        #p.stepSimulation()
        if self.enableCamera:
            self.camera()
        else:
            sleep(1./20.)
    
    def createDQObject(self,pose):

        """
        input: ['r','ri','rj','rk','p','pi','pj','pk']
        """
        #rotation
        r = DQ(pose[0:3])
        #translation
        p = DQ(pose[4:7])
                
        #Dual quaternion
        dualQuaternion = r + DQ.E*0.5*p*r
        
        return dualQuaternion

    def multiGraspSimulation(self):

        """
        Visualizes several grasp after each other, reading each file with poses and grip control data from data/grasp_data
        """


        path = os.getcwd()+"/data/grasp_data/"
        for filename in os.listdir(path):
            print("Current grasp:", filename)
            data = pd.read_table(path + filename,sep=",",names=['r','ri','rj','rk','p','pi','pj','pk','g'])
            rot = np.stack((data['r'].values,
                            data['ri'].values,
                            data['rj'].values,
                            data['rk'].values), axis=-1)

            tran = np.stack((data['p'].values,
                            data['pi'].values,
                            data['pj'].values,
                            data['pk'].values), axis=-1)

            grip = np.array((data['g'].values))

            for i in range(len(grip)):
                #rotation
                r = DQ(np.array(rot[i]))
                #translation
                p = DQ(np.array(tran[i]))
                
                #Dual quaternion
                target = r + DQ.E*0.5*p*r

                #Open or close gripper
                if grip[i] == 1:
                    self.gripper.openGripper()
                else:
                    self.gripper.grip()
                    pass
                #Move arm
                self.moveArmToEETarget(target,0.4)
                
            print("Grasp done, reseting simulation")
            self.pause(5)
            self.setJointPosition(self.homePos)
