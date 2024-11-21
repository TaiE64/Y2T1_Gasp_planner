import pybullet as p
import time
import math
import numpy as np
import pybullet_data

class testHand:
#--------------------------------------------------------------------------------
  def grasp(self):
    done = False
    while not done:
   
      for i in [1,4]:
        p.setJointMotorControl2(self.robot_model, i, p.POSITION_CONTROL, 
                                targetPosition=0.05, maxVelocity=1,force=1)
        #p.stepSimulation()
      p.setJointMotorControl2(self.robot_model, 7, p.POSITION_CONTROL, 
                                targetPosition=0.05, maxVelocity=1,force=2)
      done = True
    self.open = False
#--------------------------------------------------------------------------------
  def preshape(self):
    
    done = False
    while not done:
      
      for i in [2,5,8]:
        p.setJointMotorControl2(self.robot_model, i, p.POSITION_CONTROL, 
                                targetPosition=0.4, maxVelocity=2,force=1)
        #p.stepSimulation()
      done = True
    self.open = False
#--------------------------------------------------------------------------------
  def openGripper(self):
    closed = True
    iteration = 0
    while(closed and not self.open):
      joints = self.getJointPosition()
      closed = False
      for k in range(0,self.numJoints):

        #lower finger joints
        if k==2 or k==5 or k==8:
          goal = 0.9
          if joints[k] >= goal:    
            p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                    targetPosition=joints[k] - 0.05, 
                                    maxVelocity=2,force=5)   
            closed = True

        #Upper finger joints             
        elif k==6 or k==3 or k==9:
          goal = 0.9
          if joints[k] <= goal:
            p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                    targetPosition=joints[k] - 0.05,
                                    maxVelocity=2,force=5)
            closed = True

              #Base finger joints
        elif k==1 or k==4 or k == 7:
          pos = 0.9
          if joints[k] <= pos:
            p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                    targetPosition=joints[k] - 0.05,
                                    maxVelocity=2,force=5)
            closed = True

      iteration += 1
      if iteration > 10000:
        break
      p.stepSimulation()
    self.open = True
#--------------------------------------------------------------------------------
  def getJointPosition(self):
    joints = []
    for i in range(0, self.numJoints):
      joints.append(p.getJointState(self.robot_model, i)[0])
    return joints
#--------------------------------------------------------------------------------
  def set_init_finger_pose(self):
    self.reset_finger([[0, 0, 0, 0], [0, 0, 0, 0], 
                       [0, 0, 0, 0], [0.7, 0.8, 0.8, 0.8]])
#--------------------------------------------------------------------------------
  def reset_finger(self, hand_config):
    for i, finger_index in enumerate(self.fingers):
      for j, joint in enumerate(finger_index):
        p.resetJointState(self.gripperId, joint, hand_config[i][j])
#--------------------------------------------------------------------------------
  def run(self):
    clid = p.connect(p.SHARED_MEMORY)
    if (clid < 0):
      p.connect(p.GUI)

    path = "./Robots/grippers/threeFingers/sdh.urdf"

    hand_pos = [[0.1, 0, 0], 
                [0.2, 0, 0], 
                [0.3, 0, 0]]  

    hand_ori = p.getQuaternionFromEuler([0, 0, 0])

    initial_position = [0, 0, 0]
    initial_orientation = p.getQuaternionFromEuler([0, 0, 0]) 
    # or just use [0, 0, 0, 1] not to apply any extra orientation
    
    print("hand_pos 0, ", hand_pos[0])

    self.gripperId = p.loadURDF(path, initial_position, initial_orientation, 
                                globalScaling=1, useFixedBase=False)


    input("\033[31menter to continue\033[0m")

    print("gripperID: ", self.gripperId)

    p.setRealTimeSimulation(1)
    
    self.hand_base_controller = p.createConstraint(self.gripperId,
                                                      -1,
                                                      -1,
                                                      -1,
                                                      p.JOINT_FIXED,
                                                      [0, 0, 0], [0, 0, 0], hand_ori)     

     
    time.sleep(.03)

    for i in range(0, 3):

      input("\033[31menter to see the next pose\033[0m")
      
      p.changeConstraint(self.hand_base_controller, np.array([0.1*(i+1), 0, 0]), #hand_pos[i], 
                         jointChildFrameOrientation=hand_ori, maxForce=50)
      time.sleep(.03)
      hand_pose = p.getBasePositionAndOrientation(self.gripperId)[0:2]

      print("reached hand_position:---------- ", hand_pose[0])
      print("reached hand_orientation:--------- ", hand_pose[1])

    # let's visualize the world reference frame
    p.addUserDebugLine([0, 0, 0], [0.5, 0, 0], [1, 0, 0], 1, 0) # lookup the parameters of this!
    p.addUserDebugLine([0, 0, 0], [0, 0.5, 0], [0, 1, 0], 1, 0)
    p.addUserDebugLine([0, 0, 0], [0, 0, 0.5], [0, 0, 1], 1, 0)

    input("\033[31menter to grasp and lift an object\033[0m")

    #below we load an object and move the hand to a grasp pose
    hand_pose = p.getBasePositionAndOrientation(self.gripperId)[0:2]
    hand_ori = p.getQuaternionFromEuler([np.pi, 0, 0])
    
    p.changeConstraint(self.hand_base_controller, np.array(hand_pose[0]) + np.array([0.6, 0.6, 0.6]), 
                         jointChildFrameOrientation=hand_ori, maxForce=50)
    time.sleep(.01)
    p.changeConstraint(self.hand_base_controller, np.array([0, 0, 0.6]), 
                         jointChildFrameOrientation=hand_ori, maxForce=50)

    time.sleep(.05)
    p.changeConstraint(self.hand_base_controller, np.array([0, 0, 0.2]), 
                         jointChildFrameOrientation=hand_ori, maxForce=50)
    
    #open and close the fingers
    self.open = False
    self.numJoints = p.getNumJoints(self.gripperId)
    self.robot_model = self.gripperId
    
    input("\033[31menter to open the gripper\033[0m")
    self.openGripper()
    input("\033[31menter to preshape\033[0m")
    self.preshape()
    time.sleep(.05)

    #lift the object
    p.loadURDF("cube_small.urdf", [0, 0, 0], hand_ori, globalScaling=2)
   
    p.changeConstraint(self.hand_base_controller, np.array([0, 0, 0.15]), 
                         jointChildFrameOrientation=hand_ori, maxForce=50)

    input("\033[31menter to grasp\033[0m")
    self.grasp()
    input("\033[31mgrasp done\033[0m")
    input("\033[31menter to lift\033[0m")
    p.changeConstraint(self.hand_base_controller, np.array([0, 0, 0.3]), 
                         jointChildFrameOrientation=hand_ori, maxForce=50)

    input("\033[31menter to exit\033[0m")
    p.disconnect()
#--------------------------------------------------------------------------------

obj = testHand()
obj.run()