import airsim
import os
import numpy as np
import cv2
import pprint
import time
from split_image import split_image
import math

# drone size 1m x 1m

# directory to store pictures
imgDir = 'D:\AirSim\AirSim\dataCollection'
vehicle_name = "0"
DIRECTION_FACTOR = 5

# check that directory exists
isExist = os.path.exists(imgDir)
if not isExist:
    # make directory if not already there
    os.makedirs(imgDir)
    print('Created: ' + imgDir)

# set up client object to access multirotor drone
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

i=0
lowDepth = 0
imageContainer = []

# image collection loop:
while i < 1:

    # take images
    # ImageRequest(name, image_type, pixel_as_float, compress)
    response = client.simGetImages(
        [airsim.ImageRequest("0", airsim.ImageType.Scene,False, False),
        airsim.ImageRequest("0", airsim.ImageType.DepthVis, True)],
        vehicle_name)

    responseScene = response[0]
    responseDepth = response[1]

    depth = np.array(responseDepth.image_data_float,dtype=np.float32)
    depth = depth.reshape(responseDepth.height,responseDepth.width)
    depth = np.array(depth*255,dtype=np.uint8)
    depthClose = depth * 1000
    depthCloce16 = np.clip(depthClose,0,65535)
    os.chdir(imgDir)
    cv2.imwrite('DepthImage.png', depthCloce16)
    split_image('DepthImage.png',1,3,False,False)
    files = os.listdir(imgDir)

    for images in files:
        img = cv2.imread(images)
        temp = np.average(img)
        if(temp > lowDepth):
            if(len(imageContainer) >= 1):
                imageContainer.clear()
            lowDepth = temp
            imageContainer.append(images)

    picDiviation = cv2.imread(imageContainer[0])

    originPicWidth = depthCloce16.shape[1]
    width = picDiviation.shape[1]
    centerDrone = width/2
    width = originPicWidth + width + centerDrone
    meterConvertion = ((width*24000)*0.000265)+1

    # print(meterConvertion)
    distanceFromObsitcal = client.getDistanceSensorData(vehicle_name=vehicle_name).distance
    droneData = client.getGpsData()
    lidarData = client.getLidarData()
    
    client.moveByVelocityZAsync(5,0, -5, duration =2,vehicle_name=vehicle_name).join()
    # time.sleep(1)

    velocity = client.getGpsData(vehicle_name = vehicle_name)

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

    client.moveByVelocityZAsync(pyNormalized,pxNormalized, -5, duration =1,vehicle_name=vehicle_name)
    print('works')

    # print('------------Lidar Information --------------')
    # print(lidarData.pose.position.x_val)
    # print(lidarData.pose.position.y_val)
    # print(lidarData.pose.position.z_val)

    # if(droneData.longitude > 0):
    #     x = droneData.longitude + longitudeConvertion
    # else:
    #     x = droneData.longitude - longitudeConvertion

    # if(droneData.latitude > 0):
    #     y = droneData.latitude + latitudeConverstion
    # else:
    #     y = droneData.latitude - latitudeConverstion

    # xDifference = float(meterConvertion) - lidarData.pose.position.x_val
    # # print(xDifference)
    # yDifference = float(distanceFromObsitcal) - lidarData.pose.position.y_val
    # # print(droneData.latitude)
    # # print(yDifference)

    # xNormalized = (xDifference / sqrt(xDifference**2 + yDifference**2))*DIRECTION_FACTOR
    # yNormalized = (yDifference / sqrt(xDifference**2 + yDifference**2))*DIRECTION_FACTOR

    # print(xNormalized)
    # print(yNormalized)

    # client.moveByVelocityZAsync(yNormalized,xNormalized, 0, duration =1,vehicle_name=vehicle_name)

    time.sleep(1)
    i+=1


