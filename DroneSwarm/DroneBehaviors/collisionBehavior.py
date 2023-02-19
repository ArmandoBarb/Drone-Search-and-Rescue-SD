import airsim
import os
import numpy as np
import cv2
import pprint
import time
from split_image import split_image
import math

imgDir = 'dataCollection'
osImgDir = "/dataCollection"
DIRECTION_FACTOR = 5
imageContainer = []

def isCollisionNeeded(drone_name, client):
    vehicle_name = drone_name
    # print("Checking collision status for:", drone_name)
    distanceFromObsitcal = client.getDistanceSensorData(vehicle_name=vehicle_name).distance
    if (distanceFromObsitcal < 10):
        return 1
    return 0

def collisionAvoidanceVector(drone_name, client):
    # check that directory exists
    lowDepth = 0
    isExist = os.path.exists(imgDir)
    if not isExist:
        # make directory if not already there
        os.makedirs(imgDir)
        print('Created: ' + imgDir)

    vehicle_name = drone_name

    response = client.simGetImages([airsim.ImageRequest(drone_name, airsim.ImageType.DepthVis, True)],vehicle_name)

    responseDepth = response[0]

    depth = np.array(responseDepth.image_data_float,dtype=np.float32)
    depth = depth.reshape(responseDepth.height,responseDepth.width)
    depth = np.array(depth*255,dtype=np.uint8)
    depthClose = depth * 1000
    depthCloce16 = np.clip(depthClose,0,65535)
    os.chdir(imgDir)
    imageName = "DepthImage" + drone_name + ".png"
    cv2.imwrite(imageName, depthCloce16)

    # we split the image into multiple parts(3 col 1 row)
    split_image(imageName,1,3,False,False)
    files = os.listdir(os.curdir)

    # we find the average pixels in the image and we store it in the imageContainer
    for images in files:
        img = cv2.imread(images)
        temp = np.average(img)
        if(temp > lowDepth):
            if(len(imageContainer) >= 1):
                imageContainer.clear()
            lowDepth = temp
            imageContainer.append(images)

    # this is converting the pixels into meters converstion 
    picDiviation = cv2.imread(imageContainer[0])
    originPicWidth = depthCloce16.shape[1]
    width = picDiviation.shape[1]
    centerDrone = width/2
    width = originPicWidth + width + centerDrone
    meterConvertion = ((width*24000)*0.000265)+1


    distanceFromObsitcal = client.getDistanceSensorData(vehicle_name=vehicle_name).distance
    droneData = client.getGpsData()
    lidarData = client.getLidarData()
    
    # we have to play with this moveByVelo thing because for the collision algo to work the drone has to be moving 
    client.moveByVelocityZAsync(5,0, -5, duration =2,vehicle_name=vehicle_name).join()

    velocity = client.getGpsData(vehicle_name = vehicle_name)

    # math to find the x and y values to find the vectors
    theta = math.atan2(distanceFromObsitcal,meterConvertion)/math.pi*180
    cs = math.cos(theta)
    sn = math.sin(theta)

    print(velocity.gnss.velocity.x_val)
    print(velocity.gnss.velocity.y_val)
    px = velocity.gnss.velocity.x_val * cs - velocity.gnss.velocity.y_val * sn
    py = velocity.gnss.velocity.x_val * sn + velocity.gnss.velocity.y_val * cs


    pxNormalized = (px / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR
    pyNormalized = (py / math.sqrt(px**2 + py**2))*DIRECTION_FACTOR

    print(pxNormalized)
    print(pyNormalized)

    return [pxNormalized, pyNormalized]

    # client.moveByVelocityZAsync(pyNormalized,pxNormalized, -5, duration =1,vehicle_name=vehicle_name)