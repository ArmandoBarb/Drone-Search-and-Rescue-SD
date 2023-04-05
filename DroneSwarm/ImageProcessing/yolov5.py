import torch
import numpy as np
import cv2
from PIL import Image
from ImageProcessing import getInfo
from ImageProcessing import getInfoWolf
import os
import rospy
from airsim_ros_pkgs.srv import requestGPU0
from airsim_ros_pkgs.srv import requestGPU1
from airsim_ros_pkgs.srv import requestGPU2
from airsim_ros_pkgs.srv import requestGPU3

import rospy
import Constants.ros as ros
import time
GPU_SERVICE0 = ros.GPU_SERVICE0
GPU_SERVICE1 = ros.GPU_SERVICE1
GPU_SERVICE2 = ros.GPU_SERVICE2
GPU_SERVICE3 = ros.GPU_SERVICE3

def runYolov5(client, responses, cameraName, vehicleName, confidanceMin):
    global GPU_SERVICE0
    global GPU_SERVICE1
    global GPU_SERVICE2
    global GPU_SERVICE3

    responseIndex = 0

    # get response object with input image
    height, width, sceneRGB2 = getInfo.getHeightWidthArr(responses, responseIndex)

    # original image unedited
    responseString= responses[int(responseIndex)].image_data_uint8

    # Requests gpu service and sends over response string, gets response object
    gpuServiceTime = time.time()

    response = None

    if (vehicleName=="0"):
        rospy.wait_for_service(GPU_SERVICE0)
        response = rospy.ServiceProxy(GPU_SERVICE0, requestGPU0)
    elif (vehicleName=="1"):
        rospy.wait_for_service(GPU_SERVICE0)
        response = rospy.ServiceProxy(GPU_SERVICE0, requestGPU0)
    elif (vehicleName=="2"):
        rospy.wait_for_service(GPU_SERVICE0)
        response = rospy.ServiceProxy(GPU_SERVICE0, requestGPU0)
    elif (vehicleName=="3"):
        rospy.wait_for_service(GPU_SERVICE0)
        response = rospy.ServiceProxy(GPU_SERVICE0, requestGPU0)
        
    # elif (vehicleName=="1"):
    #     rospy.wait_for_service(GPU_SERVICE1)
    #     response = rospy.ServiceProxy(GPU_SERVICE1, requestGPU1)
    # elif (vehicleName=="2"):
    #     rospy.wait_for_service(GPU_SERVICE2)
    #     response = rospy.ServiceProxy(GPU_SERVICE2, requestGPU2)
    # elif (vehicleName=="3"):
    #     rospy.wait_for_service(GPU_SERVICE3)
    #     response = rospy.ServiceProxy(GPU_SERVICE3, requestGPU3)

    responseObject = response(str(responseString.decode('latin-1')), height, width)
    gpuLen = time.time() - gpuServiceTime
    # print("gpuLen:       " + str(gpuLen) + "         999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999")
    # print("Yolo still running")
    # print("gpuLen:       " + str(gpuLen) + "         999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999")

    # Set variables from response object
    success = responseObject.success
    xmin = int(responseObject.xMin)
    ymin = int(responseObject.yMin)
    xmax = int(responseObject.xMax)
    ymax = int(responseObject.yMax)
    confidence = responseObject.confidence

    maxConfidence = 0
    maxConfidenceGPS = [None, None]
    detection = 0
    maxConfidenceDetection = 0

    # cwd = os.getcwd()
    # dataDir=os.path.join(str(cwd),'yolov5Images')
    # isExist=os.path.exists(dataDir)
    dataDir = '/home/testuser/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD/DroneSwarm/yolov5Images'
    # cwd = os.getcwd()
    # dataDir=os.path.join(str(cwd),'yolov5Images')
    # dataDir = os.path.abspath('yolov5Images')
    isExist=os.path.exists(dataDir)

    if not isExist:
        # make directory if not already there
        os.makedirs(dataDir)

    dataDir_fail = '/home/testuser/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD/DroneSwarm/yolov5Images_fails'
    # cwd = os.getcwd()
    # dataDir_fail=os.path.join(str(cwd),'yolov5Images_fails')
    # dataDir_fail = os.path.abspath('yolov5Images_fails')
    isExist=os.path.exists(dataDir_fail)

    if not isExist:
        # make directory if not already there
        os.makedirs(dataDir_fail)

    #ToDo: just pass loop index adn drone name
    # or in final build just overwite
    j=0
    while os.path.exists(dataDir + "/" + ('%s' % j)+"w"+vehicleName+cameraName+"newImg.jpg"):
        j+=1

    #ToDo: just pass loop index adn drone name
    # or in final build just overwite
    k=0
    while os.path.exists(dataDir + "/" + ('%s' % k)+"w"+vehicleName+cameraName+"newImg.jpg"):
        k+=1

    validDetection = False
    passedConfidence = False
    # TODO: CHANGE WITH SUCCESS BOOL FROM ROS
    if(success):
        validDetection = True
        confidence = responseObject.confidence
        # if confidence is high enough use for GPS estimation
        if(confidence >= confidanceMin):
            passedConfidence=True
            
            print("Found a target!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

            start_point = (xmin, ymin)
            end_point = (xmax, ymax)
            newImag = cv2.rectangle(sceneRGB2, start_point, end_point, (0, 255, 0), 2)
            origImgPath = dataDir + "/" + ('%s' % j)+"w"+vehicleName+cameraName+"origImg.jpg"
            newImgPath = dataDir + "/" + ('%s' % j)+"w"+vehicleName+cameraName+"newImg.jpg"

            print(origImgPath)
            print(newImgPath)

            h=0

            while(not (os.path.exists(origImgPath) and os.path.exists(newImgPath))):
                print(str(h))
                # save new image only with highest confidence detection
                cv2.imwrite(origImgPath, sceneRGB2)
                cv2.imwrite(newImgPath, newImag)
                h+=1

            #print("------------------------------------------------------------------------------------------------------------------")
            # use bb dimensions/location for GPS estimation
            alt, lat, lon = getInfoWolf.getWolfGPSEstimate(client, responses, vehicleName, cameraName, xmin, ymin, xmax, ymax)
            #print("\tWOLF ESTIMATE: "+str(alt)+" alt, " + str(lat) + " lat, " + str(lon) + " lon")
            maxConfidenceGPS[1]=lat
            maxConfidenceGPS[0]=lon
            #print("------------------------------------------------------------------------------------------------------------------")

            # write corresponding text file
            with open(dataDir + "/" + ('%s' % j)+"w"+vehicleName+cameraName+"GPSEstimate.txt", 'w') as f:
                # f.write(str(resultsPandas))
                f.write("\n\tMax Confidence: "+str(confidence))
                f.write("\n\tMax Confidence Estimate:"+ str(lat) + " lat, " + str(lon) + " lon")
                f.close()

        else:
            passedConfidence=False
            start_point = (xmin, ymin)
            end_point = (xmax, ymax)
            newImag = cv2.rectangle(sceneRGB2, start_point, end_point, (0, 255, 0), 2)
            origImgPath = dataDir_fail + "/" + ('%s' % j)+"w"+vehicleName+cameraName+"origImg.jpg"
            newImgPath = dataDir_fail + "/" + ('%s' % j)+"w"+vehicleName+cameraName+"newImg.jpg"

            print(origImgPath)
            print(newImgPath)

            h = 0

            while(not (os.path.exists(origImgPath) and os.path.exists(newImgPath))):
                print(str(h))
                # save new image only with highest confidence detection
                cv2.imwrite(origImgPath, sceneRGB2)
                cv2.imwrite(newImgPath, newImag)
                h+=1

            #print("------------------------------------------------------------------------------------------------------------------")
            # use bb dimensions/location for GPS estimation
            alt, lat, lon = getInfoWolf.getWolfGPSEstimate(client, responses, vehicleName, cameraName, xmin, ymin, xmax, ymax)
            #print("\tWOLF ESTIMATE: "+str(alt)+" alt, " + str(lat) + " lat, " + str(lon) + " lon")
            maxConfidenceGPS[1]=lat
            maxConfidenceGPS[0]=lon
            #print("------------------------------------------------------------------------------------------------------------------")

            # write corresponding text file
            with open(dataDir_fail + "/" + ('%s' % k)+"w"+vehicleName+cameraName+"GPSEstimate.txt", 'w') as f:
                # f.write(str(resultsPandas))
                f.write("\n\tMax Confidence: "+str(confidence))
                f.write("\n\tMax Confidence Estimate:"+ str(lat) + " lat, " + str(lon) + " lon")
                f.close()

    return maxConfidenceGPS, validDetection, passedConfidence