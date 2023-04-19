import airsim
import os
import numpy as np
import cv2
import pprint
import time
from split_image import split_image
# import Constants.configDrones as configDrones
import math
from contextlib import contextmanager
import sys, os

@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        sys.stdout = devnull
        try:  
            yield
        finally:
            sys.stdout = old_stdout

def setUpLidar(client,vehicle_name):
    image_type = airsim.ImageType.Scene
    tree = "new_tree*"
    client.simSetDetectionFilterRadius("0", image_type, 1200) 
    client.simAddDetectionFilterMeshName("0", image_type, tree) 

def parse_lidarData(data):

    np.set_printoptions(threshold=sys.maxsize)

    # reshape array of floats to array of [X,Y,Z]
    points = np.array(data.point_cloud, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0]/3), 3))
       
    return points

def getVelo(treeInfo,droneInfo,DIRECTION_FACTOR):

    xDifference = float(droneInfo.gnss.geo_point.longitude) - treeInfo.geo_point.longitude
    yDifference = float(droneInfo.gnss.geo_point.latitude) - treeInfo.geo_point.latitude

    velX = (xDifference / math.sqrt(xDifference**2 + yDifference**2))*DIRECTION_FACTOR
    velY = (yDifference / math.sqrt(xDifference**2 + yDifference**2))*DIRECTION_FACTOR

    return velX,velY

def getLidarSensorInfo(client,vehicle_name):

    lidarInfo = client.getLidarData()
    parsedLidarInfo = parse_lidarData(lidarInfo)

    # print(lidarInfo)

    return parsedLidarInfo, lidarInfo.segmentation

def collisionAlgo(client,imgDir,vehicle_name,closestObjectDistance,DIRECTION_FACTOR,closestTree):
    i=0
    lowDepth = 0
    tempSecondChoice = 0
    imageContainer = []
    secondLowDepth = 0
    secondChoiceContainer = []
    velocity = client.getGpsData(vehicle_name = vehicle_name)
    #     # image collection loop:
    #     # we have to test out how many pictures it should take so the while loop can end
    #     # take images
    #     # ImageRequest(name, image_type, pixel_as_float, compress)
    # response = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthVis, True)],vehicle_name)
    # responseDepth = response[0]

    # depth = np.array(responseDepth.image_data_float,dtype=np.float32)
    # depth = depth.reshape(responseDepth.height,responseDepth.width)
    # depth = np.array(depth*255,dtype=np.uint8)
    # depthClose = depth * 1000
    # depthCloce16 = np.clip(depthClose,0,65535)

    # os.chdir(imgDir)
    # cv2.imwrite('DepthImage.png', depthCloce16)

    # # we split the image into multiple parts(3 col 1 row)
    # with suppress_stdout():
    #     split_image('DepthImage.png',1,3,False,False)     
               
    # files = os.listdir(imgDir)
    treeWidth = (456/100) + 2
    droneWidth = 2
    # treeWidth = 1.75
    # velX,velY = getVelo(treeWidth, closestObjectDistance,DIRECTION_FACTOR)

    velX,velY = getVelo(closestTree, velocity, DIRECTION_FACTOR)

    # velX = math.cos(velX) - math.sin(velY)
    # velY = math.sin(velX) + math.cos(velY)
            
    # # print("Vely",velY)
    # # print("Velx",velX)

    # if(closestObjectDistance < 4):
    #     velX = velX * (treeWidth+2)
    #     velY = velY * (treeWidth+2)
    # else:
    velX = velX * droneWidth
    velY = velY * droneWidth

    return [velY, velX]

def getDistance(client,vehicle_name,treeInfo,droneInfo):

    xDifference = float(droneInfo.gnss.geo_point.longitude) - treeInfo.geo_point.longitude
    yDifference = float(droneInfo.gnss.geo_point.latitude) - treeInfo.geo_point.latitude

    distance = math.sqrt(xDifference**2 + yDifference**2)

    # print("Getting Distance to object!")

    return distance 

def collisionAvoidanceCheck(client, vehicle_name, threshhold):
    # tweakDronePath(client, vehicle_name)
    # repulsion(client, vehicle_name, DIRECTION_FACTOR)
    setUpLidar(client,vehicle_name)
    image_type = airsim.ImageType.Scene
    shortestDistance = 1000
    closestTree = 0
    trees = client.simGetDetections("0", image_type)
    info = client.getGpsData(vehicle_name = vehicle_name)

    if trees:
        for tree in trees:
            distance = getDistance(client, vehicle_name,tree,info)
            # print("This is distance")
            # print(distance)
            # print("-------------")
            if(shortestDistance > distance):
                closestTree = tree
                closestTreeName = tree.name
                shortestDistance = distance
                # print("this is the shortest distance")
                # print(vehicle_name)
                # print(shortestDistance)
                # print("-------------")

        # print("Slight :", tempSlightDeviation , "Closest Object:", closestObjectDistance)
    
    if (shortestDistance < 8):
        # trees = client.simClearDetectionMeshNames("1",image_type)
        return True, shortestDistance, closestTree,closestTreeName 
    else:
        return False, None , None, None

def setupCollisionDirectory(vehicle_name):
    # directory to store pictures
    imgDir = os.path.abspath("..\collisionDetectionImages"+str(vehicle_name))

    # check that directory exists
    isExist = os.path.exists(imgDir)
    if not isExist:
        # make directory if not already there
        os.mkdir(imgDir)
        # print('Created: ' + imgDir)

    return imgDir

def start():
    vehicle_name = "0"
    # directory to store pictures
    DIRECTION_FACTOR = 5
    imgDir = setupCollisionDirectory(vehicle_name)                   
    # set up client object to access multirotor drone
    client = airsim.MultirotorClient("10.171.204.221")
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    setUpLidar(client,vehicle_name)   

    while(1):
        vector = [0, 0]
        threshhold = 16
        velY = 5
        velX = 0
        doCollision, closestObjectDistance, closestTree = collisionAvoidanceCheck(client, vehicle_name, threshhold)

        if(doCollision):
            vector = collisionAlgo(client,imgDir,vehicle_name,closestObjectDistance,DIRECTION_FACTOR,closestTree)
        else:
            vector = [5, 0]
        
        client.moveByVelocityZAsync(vector[0], vector[1], -3, duration=1,vehicle_name=vehicle_name)

# start()