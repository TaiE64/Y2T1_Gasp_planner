#This file is only for testing the Robot class
import numpy as np
import os,sys
import pybullet as p
import math
sys.path.insert(0, 'utils/')
from Robot import Robot 
import dqrobotics 
from dqrobotics import robot_modeling 
from dqrobotics.utils import DQ_Geometry
from dqrobotics import *

RobotData = {"arm": "Kuka", "gripper": 0, "robot_start_pos": [0, 0, 0], "robot_start_orientation_euler": [0, 0, 0], "K": 0.5}
physicsClient = p.connect(p.GUI)

robotKuka = Robot({"arm": "Kuka","gripper": 0,"robot_start_pos": [0, 0, 0],"robot_start_orientation_euler": [0, 0, 0],"K": 0.5, "camera": True, "camera_pos": [0,0,0]})
robotUR = Robot({"arm": "UR10", "gripper": 0, "robot_start_pos": [0, 0, 0],"robot_start_orientation_euler": [0, 0, 0],"K": 0.5, "camera": True, "camera_pos": [0,0,0]})
robotYumi = Robot({"arm": "Yumi","gripper": 0,"robot_start_pos": [0, 0, 0],"robot_start_orientation_euler": [0, 0, 0],"K": 0.5, "camera": True, "camera_pos": [0,0,0]})

def testRobot():
    
    assert robotKuka.name == "Kuka", "Robot class not working correctly"
    assert robotUR.name == "UR10", "UR10 class not working correctly"
    assert robotYumi.name == "Yumi", "Yumi class not working correctly"

def testmoveArmToEETarget():

    """ Test Kuka movement """
    xd = robotKuka.SerialManipulator.fkm(np.array([0,0,0,pi/4,0,-pi/2,0]))
    robotKuka.moveArmToEETarget(xd,0.2)

    theta = np.zeros(robotKuka.numJoints)
    for j in range(robotKuka.numJoints):
        theta[j] = p.getJointState(robotKuka.robot_model, j)[0]
    
    xTrue = robotKuka.SerialManipulator.fkm(theta)
    
    err = vec4(xTrue-xd)

    assert  np.allclose(err,np.zeros(len(err)),0.1,0.2), "Kuka does not move properly!"

    """ Test UR10 movement """
    xd = robotUR.SerialManipulator.fkm(np.array([0,0,pi/4,0,-pi/2,0]))
    robotUR.moveArmToEETarget(xd,0.2)

    theta = np.zeros(robotUR.numJoints)
    for j in range(robotUR.numJoints):
        theta[j] = p.getJointState(robotUR.robot_model, j)[0]
    
    xTrue = robotUR.SerialManipulator.fkm(theta)
    
    err = vec4(xTrue-xd)

    assert  np.allclose(err,np.zeros(len(err)),0.1,0.2), "UR10 does not move properly!"

    """ Test Yumi movement """
"""
    xd = robotYumi.SerialManipulator.fkm(np.array([0, 0, 0, 0, 0, 0, 0]))
    robotYumi.moveArmToEETarget(xd,0.2)

    theta = np.zeros(robotYumi.numJoints)
    for j in range(robotYumi.numJoints):
        theta[j] = p.getJointState(robotYumi.robot_model, j)[0]
    
    xTrue = robotYumi.SerialManipulator.fkm(theta)
    
    err = vec4(xTrue-xd)

    assert  np.allclose(err,np.zeros(len(err)),0.1,0.2), "Yumi does not move properly!"
"""
def testsetJointPosition():
    """ Testing setting joint position for Kuka"""
    target_pos = np.array([0, -(math.pi/2), 0, -(math.pi/2), 0, -(math.pi/2), 0])
    robotKuka.setJointPosition(target_pos)

    theta = np.zeros(robotKuka.numJoints)
    for j in range(robotKuka.numJoints):
        theta[j] = p.getJointState(robotKuka.robot_model, j)[0]

    assert np.allclose(theta, target_pos,0.0001,0.00001), "setJointPostion does not work for Kuka"

    """ Testing setting joint position for UR10"""

    target_pos = np.array([0, -(math.pi/2), 0, -(math.pi/2), 0, -(math.pi/2)])
    robotUR.setJointPosition(target_pos)

    theta = np.zeros(robotUR.numJoints)
    for j in range(robotUR.numJoints):
        theta[j] = p.getJointState(robotUR.robot_model, j)[0]
    
    assert np.allclose(theta, target_pos,0.0001,0.00001), "setJointPostion does not work for UR10"
    """

    target_pos = np.array([0, -(math.pi/2), 0, -(math.pi/2), 0, -(math.pi/2), 0])
    robotYumi.setJointPosition(target_pos)

    theta = np.zeros(robotYumi.numJoints)
    for j in range(robotYumi.numJoints):
        theta[j] = p.getJointState(robotYumi.robot_model, j)[0]
    
    assert np.allclose(theta, target_pos,0.0001,0.00001), "setJointPostion does not work for Yumi"
    """