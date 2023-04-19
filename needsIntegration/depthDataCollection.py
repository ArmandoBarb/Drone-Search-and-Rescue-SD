import airsim
import os
import numpy as np
import cv2
import pprint
import time
from split_image import split_image
# import Constants.configDrones as configDrones
import math

# drone size 1m x 1m
def getDistanceXConeArray(client,vehicle_name):

    distanceXConeArray = [client.getDistanceSensorData(distance_sensor_name="Front",vehicle_name=vehicle_name).distance,
                    client.getDistanceSensorData(distance_sensor_name="Left(-5)",vehicle_name=vehicle_name).distance,
                    client.getDistanceSensorData(distance_sensor_name="Right(5)",vehicle_name=vehicle_name).distance,
                    client.getDistanceSensorData(distance_sensor_name="Left(-6.5)",vehicle_name=vehicle_name).distance,
                    client.getDistanceSensorData(distance_sensor_name="Right(6.5)",vehicle_name=vehicle_name).distance,
                    client.getDistanceSensorData(distance_sensor_name="Left(-8.5)",vehicle_name=vehicle_name).distance,
                    client.getDistanceSensorData(distance_sensor_name="Right(8.5)",vehicle_name=vehicle_name).distance,]

    # for distance in distanceXConeArray:
    #     print(distance)

    return distanceXConeArray

def getDistanceCautionArray(client,vehicle_name):

    distanceCautionArray = [client.getDistanceSensorData(distance_sensor_name="Left(-8.5)",vehicle_name=vehicle_name).distance,
                    client.getDistanceSensorData(distance_sensor_name="Right(8.5)",vehicle_name=vehicle_name).distance,
                    client.getDistanceSensorData(distance_sensor_name="Close_Left",vehicle_name=vehicle_name).distance,
                    client.getDistanceSensorData(distance_sensor_name="Close_Right",vehicle_name=vehicle_name).distance,
                    client.getDistanceSensorData(distance_sensor_name="Up(5)",vehicle_name=vehicle_name).distance,
                    client.getDistanceSensorData(distance_sensor_name="Up(10)",vehicle_name=vehicle_name).distance]

    # print(distanceCautionArray[2])
    # print(distanceCautionArray[3])

    return distanceCautionArray

def getSideSensors(client,vehicle_name):

    sideSensors = [client.getDistanceSensorData(distance_sensor_name="Left_Side",vehicle_name=vehicle_name).distance,
                    client.getDistanceSensorData(distance_sensor_name="Right_Side",vehicle_name=vehicle_name).distance]

    # print(sideSensors)

    return sideSensors

def getVelo(x,y,DIRECTION_FACTOR):

    velY = math.sqrt((0 - 0)**2 + ((y)-0)**2)
    velX = math.sqrt((x - 0)**2 + (0-0)**2)

    velY = velY/DIRECTION_FACTOR
    velX = velX/DIRECTION_FACTOR

    return velX,velY

def collisionAlgo(client,imgDir,vehicle_name,closestObjectDistance,DIRECTION_FACTOR):
    i=0
    lowDepth = 0
    imageContainer = []
        # image collection loop:
        # we have to test out how many pictures it should take so the while loop can end
        # take images
        # ImageRequest(name, image_type, pixel_as_float, compress)
    response = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthVis, True)],vehicle_name)
    responseDepth = response[0]

    depth = np.array(responseDepth.image_data_float,dtype=np.float32)
    depth = depth.reshape(responseDepth.height,responseDepth.width)
    depth = np.array(depth*255,dtype=np.uint8)
    depthClose = depth * 1000
    depthCloce16 = np.clip(depthClose,0,65535)

    os.chdir(imgDir)
    cv2.imwrite('DepthImage.png', depthCloce16)

    # we split the image into multiple parts(3 col 1 row)
    split_image('DepthImage.png',1,3,False,False)
    files = os.listdir(imgDir)

    # we find the average pixels in the image and we store it in the imageContainer
    for images in files:
        img = cv2.imread(images)
        temp = np.average(img)
        if(temp > lowDepth):
            if(len(imageContainer) >= 1):
                imageContainer.clear()
            lowDepth = temp
            imageContainer.append(images)
    treeWidth = (242.7/100) + 1

    velX,velY = getVelo(treeWidth, closestObjectDistance,DIRECTION_FACTOR)

    if(imageContainer[0].__contains__('0')):
        velX = -velX

    # if(closestObjectDistance < 5):
    #     client.moveByVelocityZAsync(0,velX, 0, duration=DIRECTION_FACTOR,vehicle_name=vehicle_name)
    # else:
    return [velY, velX]

def repulsion(client,vehicle_name,DIRECTION_FACTOR):

    sideSensors = getSideSensors(client, vehicle_name)

    if(sideSensors[0] < 0.8):
        difference = 0.8-sideSensors[0]
        # print(difference)
        velX,velY = getVelo(difference, 0, DIRECTION_FACTOR)

    if(sideSensors[1] < 0.8):
        difference = 0.8-sideSensors[1]
        # print(difference)
        velX,velY = getVelo(difference, 0, DIRECTION_FACTOR)
        velX = -velX

    client.moveByVelocityZAsync(velY,velX, 0, duration=DIRECTION_FACTOR,vehicle_name=vehicle_name)

def tweakDronePath(client,vehicle_name):
    distanceCautionArray = getDistanceCautionArray(client, vehicle_name)

    if(distanceCautionArray[2] < 2):
        velX,velY = getVelo(distanceCautionArray[2], 0, 1)
        client.moveByVelocityZAsync(velY,velX, 0, duration=1,vehicle_name=vehicle_name)


    if(distanceCautionArray[3] < 2):
        velX,velY = getVelo(distanceCautionArray[3], 0, 1)
        velX = -velX
        client.moveByVelocityZAsync(velY,velX, 0, duration=1,vehicle_name=vehicle_name)

def collisionAvoidanceCheck(client, vehicle_name, threshhold):
    # tweakDronePath(client, vehicle_name)
    # repulsion(client, vehicle_name, DIRECTION_FACTOR)
    distanceXConeArray = getDistanceXConeArray(client, vehicle_name)
    closestObjectDistance = 50

    for distance in distanceXConeArray:
        if(closestObjectDistance > distance):
                closestObjectDistance = distance 

    if (closestObjectDistance < threshhold):
        return True, closestObjectDistance
    else:
        return False, None

def setupCollisionDirectory(vehicle_name):
    # directory to store pictures
    imgDir = os.path.abspath("..\depthCollection"+str(vehicle_name))

    # check that directory exists
    isExist = os.path.exists(imgDir)
    if not isExist:
        # make directory if not already there
        os.mkdir(imgDir)
        print('Created: ' + imgDir)

    return imgDir

def start():
    vehicle_name = "0"
    # directory to store pictures
    DIRECTION_FACTOR = 5

    imgDir = setupCollisionDirectory(vehicle_name)

    # set up client object to access multirotor drone
    client = airsim.MultirotorClient("10.171.204.218")
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    while(1):
        vector = [0, 0]

        threshhold = 16
        velY = 5
        velX = 0
        doCollision, closestObjectDistance = collisionAvoidanceCheck(client, vehicle_name, threshhold)
        if(doCollision):
            vector = collisionAlgo(client,imgDir,vehicle_name,closestObjectDistance,DIRECTION_FACTOR)
        else:
            vector = [5, 0]
        
        client.moveByVelocityZAsync(vector[0], vector[1], -3, duration=1,vehicle_name=vehicle_name)


start()