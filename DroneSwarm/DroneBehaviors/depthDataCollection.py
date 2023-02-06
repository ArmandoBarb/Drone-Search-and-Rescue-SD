import airsim
import os
import numpy as np
import cv2
import pprint
import time
from split_image import split_image

MIN_DEPTH_METERS = 0
MAX_DEPTH_METERS = 100

# directory to store pictures
imgDir = 'D:\AirSim\AirSim\dataCollection'
vehicle_name = '0'

def hoverDrones():
    client.hoverAsync(vehicle_name= vehicle_name)

# check that directory exists
isExist = os.path.exists(imgDir)
if not isExist:
    # make directory if not already there
    os.makedirs(imgDir)
    print('Created: ' + imgDir)

# set up client object to access multirotor drone
client = airsim.MultirotorClient()

# connect to AirSim simulator
client.confirmConnection()

i=0
lowDepth = 0

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
        print(temp)
        if(temp > lowDepth):
            lowDepth = temp
            print(images)
            print(lowDepth)
        

    cv2.imshow("Depth viz", depthCloce16)
    cv2.waitKey(0)

    # #for image depth you need to convert the Image into a 2d array 
    # imageDepth = airsim.list_to_2d_float_array(responseDepth.image_data_float, responseDepth.width, responseDepth.height)
    
    # #you have to reshape it to fit the paramenters of CV2.
    # imageDepth = imageDepth.reshape(responseDepth.height, responseDepth.width, 1)

    # #I used the max and the min of the sensor view to calculate the cameras view.
    # depth_8bit = np.interp(imageDepth, (MIN_DEPTH_METERS, MAX_DEPTH_METERS), (0, 255))
    # os.chdir(imgDir)
    # cv2.imwrite('normalDepthImage'+ str(i) + '.png', depth_8bit.astype('uint8'))

    # imageDepthClose = imageDepth * 1000
    # depth_16bit = np.clip(imageDepthClose, 0, 65535)
    # cv2.imwrite('closerDepthImage'+ str(i) +'.png', depth_16bit.astype('uint16'))

    # SceneImg = np.frombuffer(responseScene.image_data_uint8, dtype = np.uint8)
    # SceneRGB = SceneImg.reshape(responseScene.height, responseScene.width, 3)

    # airsim.write_png(os.path.normpath(imgDir + '\imgScene' + str(i) +'.png'),SceneRGB)


    print('Retrieved scene images: ', len(response))

    time.sleep(1)
    i+=1

hoverDrones()

