# import rospy
# import json
# # import Constants
import Constants.ros as ros
import rospy
import os
import torch
import numpy as np
import pandas
#from std_msgs.msg import String
#from airsim_ros_pkgs.msg import droneData
#from airsim_ros_pkgs.srv import getDroneData, getDroneDataResponse
# from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.srv import requestGPU1

global MODEL

# Environmental Variables
# ros: services
GPU_SERVICE1 = ros.GPU_SERVICE1
isModelLoaded = False

# Main Process Start ----------------------------------------------
def startYoloGPU1():
    global isModelLoaded
    global MODEL
    print("StartingYoloGPU"+str(1))
    # Create node for "ProximityWolf"
    nodeName = "YoloGPU"+str(1)
    rospy.init_node(nodeName, anonymous = True)

    # start yolo GPU
    cwd = os.getcwd()
    yoloPT = os.path.join(str(cwd), 'best_coco.pt')

    # Model is loaded global to be used by service functions
    try:
        print("Loading model")
        MODEL = torch.hub.load('ultralytics/yolov5', 'custom', path=yoloPT, trust_repo=True)
        MODEL.cuda()
        print("Model loaded")
        isModelLoaded = True
    except:
        isModelLoaded = False

    # Spins up gpu service
    print("Starting gpu service")
    startGPUService()

# Main Process end -----------------------------------------------

# Starts of gpu service for handling requests, requests are then done through the handleGPU function
def startGPUService():
    serviceName = GPU_SERVICE1
    service = rospy.Service(serviceName, requestGPU1, handleGPU)
    rospy.spin()

# # TODO: handle data retrieval for service calls
def handleGPU(request):
    global isModelLoaded
    global MODEL

    responseString = request.responseString

    # Send back gpu model status if reponse string is empty
    if (responseString == ""):
        # print("Got the service request")
        yoloRepo = '/home/testuser/.cache/torch/hub/ultralytics_yolov5_master/detect.py'

        # Checks if the yolo repo exists and load, return true if so
        if (os.path.exists(yoloRepo)):
            print("We found the path")
            return (True, 0, 0, 0, 0, 0)
        # False if repo does not exist
        else:
            # print("We did not find the path")
            return (False, 0, 0, 0, 0, 0)
        # TODO:Send back is model loaded status in success bool

    # Else we handle the response string for processing
    else:
        # convert string to byte string
        responseString = responseString.encode('latin-1')

        height = request.height
        width = request.width

        segArr = np.fromstring(responseString, dtype=np.uint8)
        sceneRGB1 = segArr.reshape(height, width, 3)

        #print("RESULTS yolov5:")
        MODEL.classes = [0] # detect only for person class (0)
        results=MODEL(sceneRGB1)
        #results.print()

        # get the bounding boxes and confidence scores for single image
        validDetection = False
        resultsPandas = results.pandas().xyxy[0]
        confidenceArr = resultsPandas.confidence

        # Check if we found no bounding boxes, if so return false with empty values
        if (len(confidenceArr) == 0):
            return (False, 0, 0, 0, 0, 0)
            # TODO:Send back false in success bool


        # TODO: Send over object with this stuff
        success = True
        xMin = resultsPandas.xmin[0]
        yMin = resultsPandas.ymin[0]
        xMax = resultsPandas.xmax[0]
        yMax = resultsPandas.ymax[0]
        confidence = resultsPandas.confidence[0]
        # print("GOT A BOUNDING BOX")
        
        return (success, xMin, yMin, xMax, yMax, confidence)
        # load yolo model

