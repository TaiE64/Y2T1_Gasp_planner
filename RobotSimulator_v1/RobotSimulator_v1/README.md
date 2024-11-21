## Chalmers Robot-Simulator:
A simple, fast and configurable robot simulator for the robots available at chalmers university of technology.
This includes the robots Kuka iiwa, UR10 and Yumi. With the grippers RG6, shadow hand and a threefinger gripper (name unknown)

## First time setup 
The first step is to create a virtual environment for the project

### Windows
Dowload project and place at suitable place, go into the project folder and run
```
FirstSetup.bat
```
This creates a virtualenv in the project folder and installs all requirements.
This includes the packages:
*Pybullet, dqrobotics, numpy, matplotlib, pynput, termcolor, pyglm, plotly, scikit-image, open3d, pytest, pyyaml*

###  other systems
Create your own virtual env with the desired packages as:
```
venv <Name of environment>
<activate environment>
pip install -r requirements.txt
```
This creates a virtualenv and installs all requirements.
This includes the packages:
*Pybullet, dqrobotics, numpy, matplotlib, pynput, termcolor, pyglm, plotly, scikit-image, open3d, pytest, pyyaml*
NOTE: If not running on windows one manually has to set which python environment that shall be used since the RobotSimulator.bat will not work.


## Starting Robot Simulator

### Windows
Change the parameters in file parameters.yaml to configure the simulation and then press
```
RobotSimulator.bat
```
This will start the simulator.

### other systems
If not using Windows one can run the file RobotSimulator.py in any desired editor or from terminal (make sure to be in the project folder)
NOTE: One manually has to set which python environment that shall be used, for windows this is done automatically in RobotSimulator.bat

## Parameter file
The environment of the simulator is mainly configured in the file parameters.yaml. In the parameter file robot, gripper, desired end effector position, start position and orientatation and more can be configured. This must be done before running the simulator! 

### Robots and grippers
Currently the available robots are Kuka, ur10 and Yumi (This can be set in the parameter file). The available grippers are threefinger, shadowHand, RG6 and yumi.
All robots can be combined with the different grippers except for Yumi, the Yumi robot and gripper can only be used with each other.

A simple example to start the simulator with the Kuka robot using the shadow hand gripper and a camera mounted on it.
```
- robot:
    arm: Kuka
    gripper: shadowHand
    robot_start_pos: [0, 0, 0]
    robot_start_orientation_euler: [0, 0, -1.57]
    K: 0.5
    camera: True
```

## Using the Simulator
When started the simulator shows the Robot in its inital position and orientation, the user can then interact with the environment through several buttons as seen below.

<img src=images/Simulator_buttons.png width="400" height="350"/>

### Buttons
1. Resets the robot to its initial position
2. The end effector is moved to a predefined target (in parameter file) with the inbuilt controller, K value can also be tuned in parameter file.
3. NOT YET IMPLEMENTED, in a future realse it will plan a path to a given target with obstacle avoidance. Might also visualize the path
4. NOT YET IMPLEMENTED, will follow the planed path from button 3
5. Will change the force of the gripper when executing forcecontrol (Needs further development)
6. Grips the object with the predefined motions (given in a dat file linked in parameter file), further development would include the given force.
7. Opens the gripper
8. Spawns a interactable table and places the robot on it, can only be done once.
9. Spawns a interactable object, only one object available at this time but more can be added in the future, the user can tehn choose object in the parameter file.
10. Takes a picture of the current cameraview
11. Saves the current pose of the robot, this can be done any amount of times, the poses can then be followed in one combined path with button 12.  
12. Goes to each of the saved poses with the built in controller to simulate a possible path of the robot.
13. Creates a point cloud of the scene and shows in a new window.
14. Executes multi grasp planning, a more detailed explanation can be seen at the title multi grasp planning
15. Enables joint control, spawns one slider for each joint so that the user can manually control the robot in any wanted direction, placing the object at any wanted pose.

16. Normal cameraview
17. Depth camera
18. Segmentation image

### Grip
The grip of the robot is dependent of a .dat file with the wanted joint positions of the gripper, thus in the parameter file the user has to add the path to this file e.g 
./data/grip/threefingers.dat. With this function the user can set and experiment with any wanted grips. 
The structure of this file would then be one data point for each joint. For the three finger gripper that would mean 10 data points as:
```
    [j1,j2,j3,j4,j5,j6,j7,j8,j9,j10]
```
If one would instead choose the shadowHand gripper it would need 30 datapoints.

### Multi grasp simulation
For quick debugging and vizualisation the user can several different grasps that can be executed after eachother to quickly find faults in the paths. A .dat file with the rotation and translation for the dual quaternions of each point and if the gripper should be open or closed should be added to the projects data/grasp_data/ folder. Name and amount of grasps does not matter. Each row in the .dat file needs to have the following structure:
```
    [r,ri,rj,rk,p,pi,pj,pk,g]
```
Where r is rotation, p is translation and g is 1 = Open or 0 = closed. The row number is unlimited so any amount of points can be added after eachother. When one grasp is done the robot will reset itself and show the next grasp.

### Data examples
Grip control: see ./data/grip/threefingers.dat or ./data/grip/shadowHand.dat

Multi grasp planning: see ./data/grasp_data/

### NOTES
The controller might at some time be slow or abort the motion to fast, this can changed in the function moveArmToEETarget() in the robot class. The last part in this function abort the controller if it runs for to long. This threshold might vary depending of the choosen K. So the threshold can be configured in the robot class, the section that needs to be changed can be seen below: 
```
if iteration > 300:
    break
```
### Adding new robots
If one would want to add new robots to the simulator a own alternative for this must be added to the code where it loads the correct DH parameters (that the user needs to add) and the correct URDF files (that the user needs to add). All of this is done in the init part of the robot class, all other classes should be compatible with it when the DH parameters and URDF files has been added. Best way would be to look at the robot class and see how the current robots are initialized.

