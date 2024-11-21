import numpy as np
import matplotlib.pyplot as plt
import pybullet
import time
import open3d as o3d
from open3d import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from IPython import embed
import glm
import pybullet_data
import os
def getPointCloud(p_env):
    #plt.ion()
    img = np.random.rand(200, 320)
    #image = plt.imshow(img, interpolation='none', animated=True, label="blah")
    #ax = plt.gca()

    camTargetPos = [0, 0, 0]
    cameraUp = [0, 0, 1]
    cameraPos = [1, 1, 1]
    pitch = -10.0
    yaw = 0
    roll = 90
    fov = 30

    upAxisIndex = 2
    camDistance =4
    pixelWidth = 320
    pixelHeight = 200
    aspect = pixelWidth / pixelHeight

    nearPlane = 0.01
    farPlane = 10

    yaw=0
    viewMatrix = p_env.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance,
                                                            yaw, pitch,
                                                            roll, upAxisIndex)

    projectionMatrix = p_env.computeProjectionMatrixFOV(
        fov, aspect, nearPlane, farPlane)
    img_arr = p_env.getCameraImage(pixelWidth,
                                        pixelHeight,
                                        viewMatrix,
                                        projectionMatrix,
                                        shadow=1,
                                        lightDirection=[1, 1, 1],
                                        renderer=p_env.ER_BULLET_HARDWARE_OPENGL)
    w = img_arr[0]  # width of the image, in pixels
    h = img_arr[1]  # height of the image, in pixels
    rgbBuffer = img_arr[2]  # color data RGB
    depthBuffer = img_arr[3]  # depth data

    print('width = %d height = %d' % (w, h))
    #image.set_data(rgbBuffer)
    #ax.plot([0])
    #plt.pause(0.01)

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
            win = glm.vec3(hh, ww, depth)
            position = glm.unProject(win, modelView, modelProj, viewport)
            #print('position: '+str(position))
            points.append([position[0], position[1], position[2]])
            temp = rgb[hh][ww]
            colors.append([temp[0], temp[1], temp[2]])
            #print(points[count])
            #print(count)
            count = count+1

    pcl = o3d.geometry.PointCloud()
    pcl.points = o3d.open3d.cpu.pybind.utility.Vector3dVector(np.array(points))
    pcl.colors = o3d.open3d.cpu.pybind.utility.Vector3dVector(np.array(colors))

    o3d.visualization.draw_geometries([pcl])

    #p_env.resetSimulation()
    #p_env.disconnect()
