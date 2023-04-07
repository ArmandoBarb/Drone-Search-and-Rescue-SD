import torch
import numpy as np
import cv2
from PIL import Image
from ImageProcessing import getInfo
from ImageProcessing import getInfoWolf
import os
import rospy
from airsim_ros_pkgs.srv import requestGPU
import rospy
import Constants.ros as ros
import time
GPU_SERVICE = ros.GPU_SERVICE

def runYolov5(client, responses, dataDir_pass, dataDir_fail, cameraName, vehicleName, confidanceMin):
    global GPU_SERVICE

    responseIndex = 0

    # get response object with input image
    height, width, sceneRGB2 = getInfo.getHeightWidthArr(responses, responseIndex)
    sceneRGB1 = np.copy(sceneRGB2)

    # original image unedited
    responseString= responses[int(responseIndex)].image_data_uint8

    # Requests gpu service and sends over response string, gets response object
    gpuServiceTime = time.time()
    rospy.wait_for_service(GPU_SERVICE)
    response = rospy.ServiceProxy(GPU_SERVICE, requestGPU)
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

    #ToDo: just pass loop index adn drone name
    # or in final build just overwite
    j=0
    while os.path.exists(dataDir_pass + "/" + ('%s' % j)+"w"+vehicleName+cameraName+"camImg.jpg"):
        j+=1

    #ToDo: just pass loop index adn drone name
    # or in final build just overwite
    k=0
    while os.path.exists(dataDir_fail + "/" + ('%s' % k)+"w"+vehicleName+cameraName+"camImg.jpg"):
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
            
            print("High Confidence Detection!!!")

            start_point = (xmin, ymin)
            end_point = (xmax, ymax)
            cv2.rectangle(sceneRGB2, start_point, end_point, (0, 255, 0), 2)
            # save new image only with highest confidence detection
            cv2.imwrite(dataDir_pass + "/" + ('%s' % j)+"w"+vehicleName+cameraName+"camImg.jpg", sceneRGB1)
            cv2.imwrite(dataDir_pass + "/" + ('%s' % j)+"w"+vehicleName+cameraName+"bbImg.jpg", sceneRGB2)

            #print("------------------------------------------------------------------------------------------------------------------")
            # use bb dimensions/location for GPS estimation
            alt, lat, lon = getInfoWolf.getWolfGPSEstimate(client, responses, vehicleName, cameraName, xmin, ymin, xmax, ymax)
            #print("\tWOLF ESTIMATE: "+str(alt)+" alt, " + str(lat) + " lat, " + str(lon) + " lon")
            maxConfidenceGPS[1]=lat
            maxConfidenceGPS[0]=lon
            #print("------------------------------------------------------------------------------------------------------------------")

            # write corresponding text file
            with open(dataDir_pass + "/" + ('%s' % j)+"w"+vehicleName+cameraName+"GPSEstimate.txt", 'w') as f:
                # f.write(str(resultsPandas))
                f.write("\n\tMax Confidence: "+str(confidence))
                f.write("\n\tMax Confidence Estimate:"+ str(lat) + " lat, " + str(lon) + " lon")
                f.close()

        else:
            passedConfidence=False
            print("Low Confidence Detection!!!")

            start_point = (xmin, ymin)
            end_point = (xmax, ymax)
            cv2.rectangle(sceneRGB2, start_point, end_point, (0, 255, 0), 2)
            # save new image only with highest confidence detection
            cv2.imwrite(dataDir_fail + "/" + ('%s' % k)+"w"+vehicleName+cameraName+"camImg.jpg", sceneRGB1)
            cv2.imwrite(dataDir_fail + "/" + ('%s' % k)+"w"+vehicleName+cameraName+"bbImg.jpg", sceneRGB2)

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