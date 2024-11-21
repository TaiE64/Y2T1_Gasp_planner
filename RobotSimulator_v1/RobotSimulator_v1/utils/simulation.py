import time
import yaml
import pybullet as p
import pkg_resources
from pkg_resources import DistributionNotFound, VersionConflict
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image
import pandas as pd
import open3d as o3d
from open3d import *
from mpl_toolkits.mplot3d import Axes3D
from IPython import embed
import glm
import pybullet_data


class sim:
    def __init__(self):
        """ 
            On init the sim loads all data from the parameter file and saves in the class as 4 different dicts:
                - self.robotParams = robotParams["robot"]
                - self.simParams = simParams["simulation"]
                - self.motionParams = motionParams["motion"]
                - self.graphicParams = graphicParams["graphics"]

            It also starts the simulator with or without GUI based on parameters, see parameters.

        """

        try:
            with open('requirements.txt', 'r') as req:
                pkg_resources.require(req)
        except DistributionNotFound:
            print('[Warning]: Missing packages ')
            sys.exit(
                '[EXIT]: System will exit, please install all requirements and run the simulator again')

        print("     ____          __            __                       ")
        print("    / __ \ ____   / /_   ____   / /_                      ")
        print("   / /_/ // __ \ / __ \ / __ \ / __/                      ")
        print("  / _, _// /_/ // /_/ // /_/ // /_                        ")
        print(" /_/ |_| \____//_.___/ \____/ \__/        __              ")
        print("    _____ (_)____ ___   __  __ / /____ _ / /_ ____   _____")
        print("   / ___// // __ `__ \ / / / // // __ `// __// __ \ / ___/")
        print("  (__  )/ // / / / / // /_/ // // /_/ // /_ / /_/ // /    ")
        print(" /____//_//_/ /_/ /_/ \__,_//_/ \__,_/ \__/ \____//_/     ")
        print("")
        input("Welcome to Chalmers Robot simulator!\nEdit the parameter file and press enter when ready...\n")

        self.loadData()

    def loadData(self):
        """
        - Load parameters from parameter.yaml.
        - start Pybullet.
        - return parameter data.
        """

        self.loadParam()

        self.startPyBullet()

        self.startVisuals()

    def loadParam(self):
        """ Load and extract data from parameter file """

        stream = open("parameters.yaml", 'r')
        robotParams, simParams, motionParams, graphicParams = yaml.safe_load(
            stream)

        self.robotParams = robotParams["robot"]
        self.simParams = simParams["simulation"]
        self.motionParams = motionParams["motion"]
        self.graphicParams = graphicParams["graphics"]

    def startPyBullet(self):
        """ start pybullet with/without GUI """

        if self.graphicParams["visuals"]:
            self.physicsClient = p.connect(p.GUI)
            p.setRealTimeSimulation(enableRealTimeSimulation=1)
        else:
            self.physicsClient = p.connect(p.DIRECT)
        # p.setTimeStep(self.simParams["timestep"])
        p.setGravity(0, 0, -9.8)

    def startVisuals(self):
        """Settings for pybullet gui and add buttons for user"""

        if self.graphicParams["visuals"]:
            if not self.graphicParams["GUI"]:
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            if self.graphicParams["quality"] == "high":
                p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
                p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            else:
                p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
                p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)

            self.resetButton = p.addUserDebugParameter(
                "Reset simulation", 1, 0, 1)
            self.startButton = p.addUserDebugParameter(
                "Move end-effector to target, no plan", 1, 0, 1)
            self.planButton = p.addUserDebugParameter(
                "Plan path to target", 1, 0, 1)
            self.pathButton = p.addUserDebugParameter(
                "Move along planned path", 1, 0, 1)
            self.GripForce = p.addUserDebugParameter("Grip force", 0, 20, 4)
            self.gripButton = p.addUserDebugParameter("Grip", 1, 0, 1)
            self.openGripperButton = p.addUserDebugParameter(
                "Open Gripper", 1, 0, 1)
            self.tableButton = p.addUserDebugParameter("Spawn table", 1, 0, 1)
            self.objButton = p.addUserDebugParameter("Spawn object", 1, 0, 1)
            self.cameraButton = p.addUserDebugParameter(
                "Take picture", 1, 0, 1)
            self.savePoseButton = p.addUserDebugParameter(
                "Get/Save pose", 1, 0, 1)
            self.followSavedPathButton = p.addUserDebugParameter(
                "Follow saved poses", 1, 0, 1)
            self.pointCloudButton = p.addUserDebugParameter(
                "Get point cloud", 1, 0, 1)
            self.multiGraspButton = p.addUserDebugParameter(
                "Multi grasp simulation", 1, 0, 1)
            self.jointControlButton = p.addUserDebugParameter(
                "Enable joint control", 1, 0, 1)

    def addjointControl(self, numJoints, currentPos):
        """Adds joint control sliders in GUI"""

        self.jointControlValue = [p.addUserDebugParameter(
            "Joint"+str(i), -3.14, 3.14, currentPos[i]) for i in range(numJoints)]
        print("[info]: Joint control added to param list")

    def getJointControlVal(self):
        """Returns current value of self joint control sliders"""

        return [p.readUserDebugParameter(x) for x in self.jointControlValue]

    def printProgressBar(self, iteration, total, prefix='', suffix='', decimals=1, length=100, fill='█', printEnd="\r"):
        """ Prints a progress bar."""

        percent = ("{0:." + str(decimals) + "f}").format(100 *
                                                         (iteration / float(total)))
        filledLength = int(length * iteration // total)
        bar = fill * filledLength + ' ' * (length - filledLength)
        print(f'\r{prefix} |{bar}| {percent}% {suffix}', end=printEnd)
        # Print New Line on Complete
        if iteration == total:
            print('')

    def getPicture(self, target=[0, 0, 0]):
        """ Shows image with matplotlib for 5 seconds """
        plt.ion()
        camTargetPos = target
        cameraUp = [0, 0, 1]
        cameraPos = self.simParams["camera_pos"]
        pitch = self.simParams["pitch"]
        yaw = self.simParams["yaw"]
        roll = self.simParams["roll"]
        fov = self.simParams["fov"]

        upAxisIndex = 2
        camDistance = 4
        pixelWidth = 1920
        pixelHeight = 1080
        aspect = pixelWidth / pixelHeight

        nearPlane = 0.01
        farPlane = 10
        viewMatrix = p.computeViewMatrix(cameraPos, camTargetPos, cameraUp)
        #viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance,yaw, pitch, roll, upAxisIndex)

        projectionMatrix = p.computeProjectionMatrixFOV(
            fov, aspect, nearPlane, farPlane)
        img_arr = p.getCameraImage(pixelWidth, pixelHeight, viewMatrix, projectionMatrix, shadow=1, lightDirection=[
                                   1, 1, 1], renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgbBuffer = img_arr[2]

        w = img_arr[0]  # width of the image, in pixels
        h = img_arr[1]  # height of the image, in pixels
        np_img_arr = np.reshape(rgbBuffer, (h, w, 4))
        np_img_arr = np_img_arr * (1. / 255.)

        try:
            plt.imshow(np_img_arr)
            plt.pause(5)  # show it for 5 secs
            plt.close()
        except:
            pass

    def getPointCloud(self, target=[0, 0, 0],extraCamPos=None):
        """ Creates point cloud of the scene using the syntetic cameras"""
        camTargetPos = target
        cameraUp = [0, 0, 1]
        cameraPos = self.simParams["camera_pos"]
        pitch = self.simParams["pitch"]
        yaw = self.simParams["yaw"]
        roll = self.simParams["roll"]
        fov = self.simParams["fov"]

        pixelWidth = 320
        pixelHeight = 200
        aspect = pixelWidth / pixelHeight
        nearPlane = 0.01
        farPlane = 1000

        viewMatrix = p.computeViewMatrix(cameraPos, camTargetPos, cameraUp)
        #viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance,yaw, pitch, roll, upAxisIndex)

        projectionMatrix = p.computeProjectionMatrixFOV(
            fov, aspect, nearPlane, farPlane)
        img_arr = p.getCameraImage(pixelWidth, pixelHeight, viewMatrix, projectionMatrix, shadow=1, lightDirection=[
                                   1, 1, 1], renderer=p.ER_BULLET_HARDWARE_OPENGL)
        w = img_arr[0]  # width of the image, in pixels
        h = img_arr[1]  # height of the image, in pixels
        rgbBuffer = img_arr[2]  # color data RGB
        depthBuffer = img_arr[3]  # depth data
        points = []

        imgH = h
        imgW = w
        stepX = 1
        stepY = 1

        viewport = glm.vec4(0, 0, w, h)

        modelView = glm.mat4(viewMatrix[0], viewMatrix[1], viewMatrix[2], viewMatrix[3],
                             viewMatrix[4], viewMatrix[5], viewMatrix[6], viewMatrix[7],
                             viewMatrix[8], viewMatrix[9], viewMatrix[10], viewMatrix[11],
                             viewMatrix[12], viewMatrix[13], viewMatrix[14], viewMatrix[15],)
        modelProj = glm.mat4(projectionMatrix[0], projectionMatrix[1], projectionMatrix[2], projectionMatrix[3],
                             projectionMatrix[4], projectionMatrix[5], projectionMatrix[6], projectionMatrix[7],
                             projectionMatrix[8], projectionMatrix[9], projectionMatrix[10], projectionMatrix[11],
                             projectionMatrix[12], projectionMatrix[13], projectionMatrix[14], projectionMatrix[15])

        colors = []
        count = 0
        rgb = np.reshape(rgbBuffer, (h, w, 4))
        rgb = rgb * (1. / 255.)
        for hh in range(0, imgH, stepX):
            for ww in range(0, imgW, stepY):
                depthImg = float(depthBuffer[hh][ww])
                depth = farPlane * nearPlane / \
                    (farPlane - (farPlane - nearPlane) * depthImg)
                win = glm.vec3(ww, h-hh, depthImg)
                if depth < farPlane:
                    position = glm.unProject(
                        win, modelView, modelProj, viewport)
                    #print('position: '+str(position))
                    points.append([position[0], position[1], position[2]])
                    temp = rgb[hh][ww]
                    colors.append([temp[0], temp[1], temp[2]])

                    count = count+1
        if extraCamPos is not None:
            
            viewMatrix = p.computeViewMatrix(extraCamPos, camTargetPos, cameraUp)
            
            projectionMatrix = p.computeProjectionMatrixFOV(
                fov, aspect, nearPlane, farPlane)
            img_arr = p.getCameraImage(pixelWidth, pixelHeight, viewMatrix, projectionMatrix, shadow=1, lightDirection=[
                                    1, 1, 1], renderer=p.ER_BULLET_HARDWARE_OPENGL)
            w = img_arr[0]  # width of the image, in pixels
            h = img_arr[1]  # height of the image, in pixels
            rgbBuffer = img_arr[2]  # color data RGB
            depthBuffer = img_arr[3]  # depth data
            modelView = glm.mat4(viewMatrix[0], viewMatrix[1], viewMatrix[2], viewMatrix[3],
                                viewMatrix[4], viewMatrix[5], viewMatrix[6], viewMatrix[7],
                                viewMatrix[8], viewMatrix[9], viewMatrix[10], viewMatrix[11],
                                viewMatrix[12], viewMatrix[13], viewMatrix[14], viewMatrix[15],)
            modelProj = glm.mat4(projectionMatrix[0], projectionMatrix[1], projectionMatrix[2], projectionMatrix[3],
                                projectionMatrix[4], projectionMatrix[5], projectionMatrix[6], projectionMatrix[7],
                                projectionMatrix[8], projectionMatrix[9], projectionMatrix[10], projectionMatrix[11],
                                projectionMatrix[12], projectionMatrix[13], projectionMatrix[14], projectionMatrix[15])

        rgb = np.reshape(rgbBuffer, (h, w, 4))
        rgb = rgb * (1. / 255.)
        for hh in range(0, imgH, stepX):
            for ww in range(0, imgW, stepY):
                depthImg = float(depthBuffer[hh][ww])
                depth = farPlane * nearPlane / \
                    (farPlane - (farPlane - nearPlane) * depthImg)
                win = glm.vec3(ww, h-hh, depthImg)
                if depth < farPlane:
                    position = glm.unProject(
                        win, modelView, modelProj, viewport)
                    #print('position: '+str(position))
                    points.append([position[0], position[1], position[2]])
                    temp = rgb[hh][ww]
                    colors.append([temp[0], temp[1], temp[2]])

                    count = count+1
        pcl = o3d.geometry.PointCloud()
        pcl.points = o3d.open3d.cpu.pybind.utility.Vector3dVector(
            np.array(points))
        pcl.colors = o3d.open3d.cpu.pybind.utility.Vector3dVector(
            np.array(colors))

        filename = "generated_pointcloud.ply"
        val = o3d.io.write_point_cloud(filename, pcl)
        print(f"Generated point cloud : {val}")
        o3d.visualization.draw_geometries([pcl])
