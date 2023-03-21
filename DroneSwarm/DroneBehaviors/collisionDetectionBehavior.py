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

# drone size 1m x 1m
def getDistanceXConeArray(client,vehicle_name):

    distanceXConeArray = [{"name": "Front(-3)",'distance':client.getDistanceSensorData(distance_sensor_name="Front(-3)",vehicle_name=vehicle_name).distance},
                    {"name": "Front(3)",'distance':client.getDistanceSensorData(distance_sensor_name="Front(3)",vehicle_name=vehicle_name).distance},
                    {"name": "Front(-2)",'distance':client.getDistanceSensorData(distance_sensor_name="Front(-2)",vehicle_name=vehicle_name).distance},
                    {"name": "Front(2)",'distance':client.getDistanceSensorData(distance_sensor_name="Front(2)",vehicle_name=vehicle_name).distance},
                    {"name": "Front(-1)",'distance':client.getDistanceSensorData(distance_sensor_name="Front(-1)",vehicle_name=vehicle_name).distance},
                    {"name": "Front(1)",'distance':client.getDistanceSensorData(distance_sensor_name="Front(1)",vehicle_name=vehicle_name).distance},
                    {"name": "Up(2.5)",'distance':client.getDistanceSensorData(distance_sensor_name="Up(2.5)",vehicle_name=vehicle_name).distance},
                    {"name": "Front",'distance':client.getDistanceSensorData(distance_sensor_name="Front",vehicle_name=vehicle_name).distance},
                    {"name": "Left(-5)",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-5)",vehicle_name=vehicle_name).distance},
                    {"name": "Right(5)",'distance':client.getDistanceSensorData(distance_sensor_name="Right(5)",vehicle_name=vehicle_name).distance},
                    {"name": "Left(-4)",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-4)",vehicle_name=vehicle_name).distance},
                    {"name": "Left(4)",'distance':client.getDistanceSensorData(distance_sensor_name="Right(4)",vehicle_name=vehicle_name).distance},
                    {"name": "Left_7",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-7)",vehicle_name=vehicle_name).distance},
                    {"name": "Right_7",'distance':client.getDistanceSensorData(distance_sensor_name="Right(7)",vehicle_name=vehicle_name).distance},
                    {"name": "Left_6",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-6)",vehicle_name=vehicle_name).distance},
                    {"name": "Right_6",'distance':client.getDistanceSensorData(distance_sensor_name="Right(6)",vehicle_name=vehicle_name).distance},]
    # for distance in distanceXConeArray:
    #     print(distance)

    return distanceXConeArray

def getSlightDeviation(client,vehicle_name):

    deviationArray = [{"name": "Left_8",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-8)",vehicle_name=vehicle_name).distance},
                    {"name": "Right_8",'distance':client.getDistanceSensorData(distance_sensor_name="Right(8)",vehicle_name=vehicle_name).distance},
                    {"name": "Left_9",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-9)",vehicle_name=vehicle_name).distance},
                    {"name": "Right_9",'distance':client.getDistanceSensorData(distance_sensor_name="Right(9)",vehicle_name=vehicle_name).distance},
                    {"name": "Left_10",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-10)",vehicle_name=vehicle_name).distance},
                    {"name": "Right_10",'distance':client.getDistanceSensorData(distance_sensor_name="Right(10)",vehicle_name=vehicle_name).distance},
                    {"name": "Left_11",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-11)",vehicle_name=vehicle_name).distance},
                    {"name": "Right_11",'distance':client.getDistanceSensorData(distance_sensor_name="Right(11)",vehicle_name=vehicle_name).distance},
                    {"name": "Left_12",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-11)",vehicle_name=vehicle_name).distance},
                    {"name": "Right_12",'distance':client.getDistanceSensorData(distance_sensor_name="Right(11)",vehicle_name=vehicle_name).distance},
                    {"name": "Left_13",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-13)",vehicle_name=vehicle_name).distance},
                    {"name": "Right_13",'distance':client.getDistanceSensorData(distance_sensor_name="Right(13)",vehicle_name=vehicle_name).distance},
                    {"name": "Left_33",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-33)",vehicle_name=vehicle_name).distance},
                    {"name": "Right_33",'distance':client.getDistanceSensorData(distance_sensor_name="Right(33)",vehicle_name=vehicle_name).distance},
                    {"name": "Left_32",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-32)",vehicle_name=vehicle_name).distance},
                    {"name": "Right_32",'distance':client.getDistanceSensorData(distance_sensor_name="Right(32)",vehicle_name=vehicle_name).distance},
                    {"name": "Left_31",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-31)",vehicle_name=vehicle_name).distance},
                    {"name": "Right_31",'distance':client.getDistanceSensorData(distance_sensor_name="Right(31)",vehicle_name=vehicle_name).distance},
                    {"name": "Left_30",'distance':client.getDistanceSensorData(distance_sensor_name="Left(-31)",vehicle_name=vehicle_name).distance},
                    {"name": "Right_30",'distance':client.getDistanceSensorData(distance_sensor_name="Right(31)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(75)",'distance':client.getDistanceSensorData(distance_sensor_name="RightSide(75)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(-75)",'distance':client.getDistanceSensorData(distance_sensor_name="LeftSide(-75)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(76)",'distance':client.getDistanceSensorData(distance_sensor_name="RightSide(76)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(-76)",'distance':client.getDistanceSensorData(distance_sensor_name="LeftSide(-76)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(77)",'distance':client.getDistanceSensorData(distance_sensor_name="RightSide(77)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(-77)",'distance':client.getDistanceSensorData(distance_sensor_name="LeftSide(-77)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(78)",'distance':client.getDistanceSensorData(distance_sensor_name="RightSide(78)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(-78)",'distance':client.getDistanceSensorData(distance_sensor_name="LeftSide(-78)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(79)",'distance':client.getDistanceSensorData(distance_sensor_name="RightSide(79)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(-79)",'distance':client.getDistanceSensorData(distance_sensor_name="LeftSide(-79)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(80)",'distance':client.getDistanceSensorData(distance_sensor_name="RightSide(80)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(-80)",'distance':client.getDistanceSensorData(distance_sensor_name="LeftSide(-80)",vehicle_name=vehicle_name).distance},]

    return deviationArray

def getSideSensors(client,vehicle_name):

    sideSensors = [ {"name": "Side(75)",'distance':client.getDistanceSensorData(distance_sensor_name="Side(75)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(-75)",'distance':client.getDistanceSensorData(distance_sensor_name="Side(-75)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(76)",'distance':client.getDistanceSensorData(distance_sensor_name="Side(76)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(-76)",'distance':client.getDistanceSensorData(distance_sensor_name="Side(-76)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(77)",'distance':client.getDistanceSensorData(distance_sensor_name="Side(77)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(-77)",'distance':client.getDistanceSensorData(distance_sensor_name="Side(-77)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(78)",'distance':client.getDistanceSensorData(distance_sensor_name="Side(78)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(-78)",'distance':client.getDistanceSensorData(distance_sensor_name="Side(-78)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(79)",'distance':client.getDistanceSensorData(distance_sensor_name="Side(79)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(-79)",'distance':client.getDistanceSensorData(distance_sensor_name="Side(-79)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(80)",'distance':client.getDistanceSensorData(distance_sensor_name="Side(80)",vehicle_name=vehicle_name).distance},
                    {"name": "Side(-80)",'distance':client.getDistanceSensorData(distance_sensor_name="Side(-80)",vehicle_name=vehicle_name).distance},
                    ]

    # print(sideSensors)

    return sideSensors

def getVelo(x,y,DIRECTION_FACTOR):

    velY = math.sqrt((0 - 0)**2 + ((y)-0)**2)
    velX = math.sqrt((x - 0)**2 + (0-0)**2)

    velY = velY/DIRECTION_FACTOR
    velX = velX/DIRECTION_FACTOR

    return velX,velY

def collisionAlgo(client,imgDir,vehicle_name,closestObjectDistance,slightDeviation,DIRECTION_FACTOR,slightFlag,sensorName):
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
    with suppress_stdout():
        split_image('DepthImage.png',1,3,False,False)     
               
    files = os.listdir(imgDir)

    # we find the average pixels in the image and we store it in the imageContainer
    for images in files:
        img = cv2.imread(images)
        temp = np.average(img)
        if(temp > lowDepth):
            if(len(imageContainer) >= 1):
                secondContainer = imageContainer
                imageContainer.clear()
            lowDepth = temp
            imageContainer.append(images)
    treeWidth = (242.7/100) + 2.5
    # halOfDrone = 2
    # velX,velY = getVelo(treeWidth, closestObjectDistance,DIRECTION_FACTOR)

    if (len(imageContainer) == 0):  
        # print("No images")
        print()
    elif(imageContainer[0].__contains__('0') and ( not ("Left" in sensorName))):
        # if (vehicle_name == '0'):
        #     print("Got a detection, picking left image: ", vehicle_name)
        velocity = client.getGpsData(vehicle_name = vehicle_name)

        if(slightFlag):
            # print("Slight Deviation")
            theta = math.atan2(treeWidth,slightDeviation)/math.pi*180
            if(slightDeviation < 4.5):
                if (theta < 0):
                    theta = theta - 2
                else:
                    theta = theta + 2
        else:
            # print("Collision")
            theta = math.atan2(treeWidth,closestObjectDistance)/math.pi*180
            if(closestObjectDistance < 7):
                if (theta < 0):
                    theta = theta - 5
                else:
                    theta = theta + 5   

        # math to find the x and y values to find the vectors

        # Adds five percent increase 
        theta = theta * -1
        cs = math.cos(theta)
        sn = math.sin(theta)

        px = velocity.gnss.velocity.x_val * cs - velocity.gnss.velocity.y_val * sn
        py = velocity.gnss.velocity.x_val * sn + velocity.gnss.velocity.y_val * cs

        pxNormalized = (px / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR
        pyNormalized = (py / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR 

        velX = pxNormalized
        velY = pyNormalized

        # if(slightDeviation < 3):
        #     velY = 0
        # elif(closestObjectDistance < 3):
        #     velY = 0

        # print("Degree turn: ", theta)
        
    elif(imageContainer[0].__contains__('1') and ( not ("Front" in sensorName))):
        # if (vehicle_name == '0'):
        #     print("Got a detection, picking center image: ", vehicle_name)
        velocity = client.getGpsData(vehicle_name = vehicle_name)
        # math to find the x and y values to find the vectors
        # print("Collision")
        if(slightFlag):
            # print("Slight Deviation")
            theta = math.atan2(treeWidth,slightDeviation)/math.pi*180
            if(slightDeviation < 4.5):
                if (theta < 0):
                    theta = theta - 2
                else:
                    theta = theta + 2
        else:
            # print("Collision")
            theta = math.atan2(treeWidth,closestObjectDistance)/math.pi*180
            if(closestObjectDistance < 7):
                if (theta < 0):
                    theta = theta - 5
                else:
                    theta = theta + 5  
    
        cs = math.cos(theta)
        sn = math.sin(theta)

        px = velocity.gnss.velocity.x_val * cs - velocity.gnss.velocity.y_val * sn
        py = velocity.gnss.velocity.x_val * sn + velocity.gnss.velocity.y_val * cs

        pxNormalized = (px / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR
        pyNormalized = (py / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR 

        velX = pxNormalized
        velY = pyNormalized

        # print("Degree turn: ", theta)

    elif(imageContainer[0].__contains__('2') and ( not ("Right" in sensorName))):
        # if (vehicle_name == '0'):
        #     print("Got a detection, picking right image: ", vehicle_name)
        velocity = client.getGpsData(vehicle_name = vehicle_name)

        # math to find the x and y values to find the vectors
        if(slightFlag):
            # print("Slight Deviation")
            theta = math.atan2(treeWidth,slightDeviation)/math.pi*180
            if(slightDeviation < 4.5):
                if (theta < 0):
                    theta = theta - 2
                else:
                    theta = theta + 2
        else:
            # print("Collision")
            theta = math.atan2(treeWidth,closestObjectDistance)/math.pi*180
            if(closestObjectDistance < 7):
                if (theta < 0):
                    theta = theta - 5
                else:
                    theta = theta + 5  

        theta = theta
        cs = math.cos(theta)
        sn = math.sin(theta)

        px = velocity.gnss.velocity.x_val * cs - velocity.gnss.velocity.y_val * sn
        py = velocity.gnss.velocity.x_val * sn + velocity.gnss.velocity.y_val * cs

        pxNormalized = (px / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR
        pyNormalized = (py / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR 

        velX = pxNormalized
        velY = pyNormalized

        # if(slightDeviation < 3):
        #     velY = 0
        # elif(closestObjectDistance < 3):
        #     velY = 0

        # print("Degree turn: ", theta)

    elif(( ("Left" in sensorName))):
       # if (vehicle_name == '0'):
        #     print("Got a detection, picking right image: ", vehicle_name)
        velocity = client.getGpsData(vehicle_name = vehicle_name)

        # math to find the x and y values to find the vectors
        if(slightFlag):
            # print("Slight Deviation")
            theta = math.atan2(treeWidth,slightDeviation)/math.pi*180
            if(slightDeviation < 4.5):
                if (theta < 0):
                    theta = theta - 2
                else:
                    theta = theta + 2
        else:
            # print("Collision")
            theta = math.atan2(treeWidth,closestObjectDistance)/math.pi*180
            if(closestObjectDistance < 7):
                if (theta < 0):
                    theta = theta - 5
                else:
                    theta = theta + 5  

        theta = theta
        cs = math.cos(theta)
        sn = math.sin(theta)

        px = velocity.gnss.velocity.x_val * cs - velocity.gnss.velocity.y_val * sn
        py = velocity.gnss.velocity.x_val * sn + velocity.gnss.velocity.y_val * cs

        pxNormalized = (px / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR
        pyNormalized = (py / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR 

        velX = pxNormalized
        velY = pyNormalized

    elif(( ("Right" in sensorName))):
        # if (vehicle_name == '0'):
        #     print("Got a detection, picking left image: ", vehicle_name)
        velocity = client.getGpsData(vehicle_name = vehicle_name)

        if(slightFlag):
            # print("Slight Deviation")
            theta = math.atan2(treeWidth,slightDeviation)/math.pi*180
            if(slightDeviation < 4.5):
                if (theta < 0):
                    theta = theta - 2
                else:
                    theta = theta + 2
        else:
            # print("Collision")
            theta = math.atan2(treeWidth,closestObjectDistance)/math.pi*180
            if(closestObjectDistance < 7):
                if (theta < 0):
                    theta = theta - 5
                else:
                    theta = theta + 5  
        
        theta = theta * -1
        cs = math.cos(theta)
        sn = math.sin(theta)

        px = velocity.gnss.velocity.x_val * cs - velocity.gnss.velocity.y_val * sn
        py = velocity.gnss.velocity.x_val * sn + velocity.gnss.velocity.y_val * cs

        pxNormalized = (px / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR
        pyNormalized = (py / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR 

        velX = pxNormalized
        velY = pyNormalized
        # Turn left

    
    elif(( ("Front" in sensorName))):
        # if (vehicle_name == '0'):
        #     print("Got a detection, picking left image: ", vehicle_name)
        velocity = client.getGpsData(vehicle_name = vehicle_name)
    
        if(slightFlag):
            # print("Slight Deviation")
            theta = math.atan2(treeWidth,slightDeviation)/math.pi*180
            if(slightDeviation < 4.5):
                if (theta < 0):
                    theta = theta - 2
                else:
                    theta = theta + 2
        else:
            # print("Collision")
            theta = math.atan2(treeWidth,closestObjectDistance)/math.pi*180
            if(closestObjectDistance < 7):
                if (theta < 0):
                    theta = theta - 5
                else:
                    theta = theta + 5   
        
        theta = theta
        cs = math.cos(theta)
        sn = math.sin(theta)

        px = velocity.gnss.velocity.x_val * cs - velocity.gnss.velocity.y_val * sn
        py = velocity.gnss.velocity.x_val * sn + velocity.gnss.velocity.y_val * cs

        pxNormalized = (px / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR
        pyNormalized = (py / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR 

        velX = pxNormalized
        velY = pyNormalizedelse
    

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

def collisionAvoidanceCheck(client, vehicle_name, threshhold,slightThresh):
    # tweakDronePath(client, vehicle_name)
    # repulsion(client, vehicle_name, DIRECTION_FACTOR)
    distanceXConeArray = getDistanceXConeArray(client, vehicle_name)

    extremeSensors = getSlightDeviation(client, vehicle_name)

    closestObjectDistance = 50
    tempSlightDeviation = 50
    tempSide = 50
    slightFlag = False
    sideFlag = False
        
    for distance in distanceXConeArray:
        if(closestObjectDistance > distance['distance']):
                sensorName = distance['name']
                closestObjectDistance = distance['distance']

    for distance in extremeSensors:
        if(tempSlightDeviation > distance['distance']):
                sensorName = distance['name']
                tempSlightDeviation = distance['distance']

    # print("Slight :", tempSlightDeviation , "Closest Object:", closestObjectDistance)
    
    if (closestObjectDistance < threshhold):
        return True, closestObjectDistance,None,False,sensorName 
    elif((slightThresh > tempSlightDeviation)):
        return True,None,tempSlightDeviation,True, sensorName
    else:
        return False, None , None , None ,None

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

# start()
