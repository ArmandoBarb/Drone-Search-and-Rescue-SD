import airsim
import os
import numpy as np
import cv2
import pprint
import time
from sklearn import preprocessing
import shutil
import math
import torch
import warnings
import Constants.configDrones as configDrones
BATCH_NAME = configDrones.BATCH_NAME

warnings.filterwarnings("ignore", category=DeprecationWarning) 

# import for making bounding boxes
#import math
#import matplotlib.pyplot as plt
#import pandas as pd

heightArr = []
widthArr = []
blue = (255, 0, 0)  # color of bounding box
thickness = 1       # bounding box thickness
pixCount = 0        # number of pixels within bounding box
droneName = ''

# replacement for math.dist
# calculates euclidian distance
def dist(P, Q):
    return (sum((pi - qi)**2 for pi, qi in zip(P, Q)))**0.5

# list of pixel clusters (a list of lists)
clusters = []

def checkSiblingClusterExists(pixelRGB, currentCluster, sceneRGB):
    if currentCluster==len(clusters):
        return False
    
    for i in range(currentCluster+1, len(clusters)):
        clusterColor = sceneRGB[clusters[i][0][0]][clusters[i][0][1]][0]
        if(pixelRGB==clusterColor):
            return True

    return False


def pixelClustering(height, width, segRGB, sceneRGB):
    global pixCount
    global clusters


    for i in range(height):
        for j in range(width):
            
            if segRGB[i][j][0] <= 255 and \
               segRGB[i][j][1] <= 255 and \
               segRGB[i][j][2] <= 255 and \
               segRGB[i][j][0] >= 254 and \
               segRGB[i][j][1] >= 254 and \
               segRGB[i][j][2] >= 254:
                pixCount = pixCount + 1

                isMatch = False
                isClustered = False
                pixel = [(i, j)]

                # found heat signature pixels
                if not clusters:
                    clusters.append(pixel)
                else:
                    clusterCount = 0
                    # not empty case
                    for cluster in clusters:
                        sibilingExists = False
                        for coord in cluster:
                            # properly place the cluster among appropriate color
                            imgPixel = segRGB[i][j][0]
                            clusterPixel = segRGB[coord[0]][coord[1]][0]
                            if imgPixel != clusterPixel:
                                break
                            else:
                                isMatch = True

                            # otherwise threshold and sort the coordinate
                            if dist([i, j], [coord[0], coord[1]]) < 100:
                                # print("in dist([i, j], [coord[0], coord[1]]) < 40")
                                # print("After dist([i, j], [coord[0], coord[1]]) < 40")
                                # print("Cluster size: ", len(cluster))
                                # print("Cluster list size: ", len(clusters))
                                # print("RGB = ", [segRGB[i][j][0], segRGB[i][j][1], segRGB[i][j][2]])
                                cluster.append(pixel[0])
                                # print("---------------------------------")
                                # print(cluster)
                                # print("---------------------------------")
                                isClustered = True
                                break
                            else:
                                # used for same heat signature for two animals
                                # but they are far apart
                                sibilingExists = checkSiblingClusterExists(clusterPixel, clusterCount, segRGB)
                                if not sibilingExists:
                                    isClustered = True
                                    clusters.append(pixel)
                                break

                        clusterCount+=1
                        if isClustered:
                            break
                    # no match for non-empty list
                    if not isMatch:
                        clusters.append(pixel)

def drawBB(sceneRGB):
    global heightArr
    global widthArr
    global clusters

    hArr = []
    wArr = []

    for cluster in clusters:
        for coord in cluster:
            if (coord[0] not in heightArr):
                heightArr.append(coord[0])

            if (coord[1] not in widthArr):
                widthArr.append(coord[1])

        # calculate centroids
        if (len(heightArr) != 0):
            # calculate y centroids
            y0 = min(heightArr) # bottom right y
            y1 = max(heightArr) # top left y

            # calculate x centroids
            x0 = min(widthArr) # top left x
            x1 = max(widthArr) # bottom right x

        # draw bounding box
        bbw = x1 - x0 # width
        start_point = (x0-math.floor(bbw*.10), y1) # top left corner (corrected offset)
        end_point = (x1, y0)                       # bottom right corner (corrected offset)
        sceneRGB = cv2.rectangle(sceneRGB, start_point, end_point, blue, thickness)

        hArr=heightArr
        wArr=widthArr

        # reset height and width arrays
        heightArr = []
        widthArr = []
        
        # print("Number of clusters: ", len(clusters))

        # empty cluster list
        clusters = []

    saveData(sceneRGB, hArr, wArr)
    pixCount = 0

    return sceneRGB

def saveData(sceneRGB2, heightArr, widthArr):
    dataDir='/home/testuser/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD-1/DroneSwarm/DataCollection'

    imgDir = dataDir + '/images'
    labelDir = dataDir + '/labels'

    if ((len(heightArr) != 0) and (pixCount >= 200)):
        # calculate y centroids
        y0 = min(heightArr)
        y1 = max(heightArr)
        yC = ((y1 - y0) / 2) + y0

        # calculate x centroids
        x0 = min(widthArr)
        x1 = max(widthArr)

        # get bbox dimensions
        bbw = x1 - x0
        bbh = y1 - y0

        x0 = x0-math.floor(bbw*.25)
        x1 = x1-math.floor(bbw*.25)
        xC = ((x1 - x0) / 2) + x0

        # print("Bounding Box Centroid (xC, yC) = " + str(xC) + " " + str(yC))

        # drawing bounding box
        start_point2 = (x0, y1) # top left corner
        end_point2 = (x1, y0)   # bottom right corner

        # print(os.path.exists(imgTrainDir+"\\" + str(j) + "imgScene.png"))

        j=0
        while os.path.exists(os.path.join(str(imgDir), droneName + "wolf" +str(j) +BATCH_NAME+ "imgScene.png")):
            j+=1

        # normalize bbox dimensions and centroids
        props = preprocessing.normalize([np.array([xC, yC, bbw, bbh])])
        print ("0" + str(props))

        # write yolo gt to text file
        with open(os.path.join(str(labelDir), droneName + "wolf" +str(j) +BATCH_NAME+ "imgScene.txt"), 'w') as f:
            f.write("0 " + str(props[0][0]) + " " + str(props[0][1]) + " " + str(props[0][2]) + " " + str(props[0][3]))

        sceneSavePath2 = os.path.join(str(imgDir), droneName + "wolf" +str(j) +BATCH_NAME+ "imgScene.png")
        
        airsim.write_png(os.path.normpath(sceneSavePath2), sceneRGB2)

def setupDirectories():
    # cwd = os.getcwd()
    # dataDir=os.path.join(str(cwd),'DataCollection')
    # isExist=os.path.exists(dataDir)

    # imgDir = os.path.join(str(dataDir), 'images')
    # labelDir = os.path.join(str(dataDir), 'labels')

    dataDir='/home/testuser/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD-1/DroneSwarm/DataCollection'
    isExist=os.path.exists(dataDir)

    imgDir = dataDir + '/images'
    labelDir = dataDir + '/labels'

    if not isExist:
        # make directory if not already there
        os.makedirs(dataDir)
        print('Created: ' + dataDir)

    # check that directory exists
    isExist = os.path.exists(imgDir)
    if not isExist:
        # make directory if not already there
        os.makedirs(imgDir)
        print('Created: ' + imgDir)

    # check that directory exists
    isExist = os.path.exists(labelDir)
    if not isExist:
        # make directory if not already there
        os.makedirs(labelDir)
        print('Created: ' + labelDir)

def processImageResponses(responses):
    responseScene = responses[1]
    responseSeg = responses[0]

    height = responses[0].height
    width = responses[0].width

    targetArr = np.fromstring(responseScene.image_data_uint8, dtype=np.uint8) # bw target before
    
    sceneArr = np.copy(targetArr) # bw target after
    sceneArr2 = np.copy(targetArr)   
    
    segArr = np.fromstring(responseSeg.image_data_uint8, dtype=np.uint8)

    # shape image
    targetRGB = targetArr.reshape(height, width, 3)
    sceneRGB = sceneArr.reshape(height, width, 3)
    sceneRGB2 = sceneArr2.reshape(height, width, 3)
    segRGB = segArr.reshape(height, width, 3)


    # cluster animal heat signatures
    pixelClustering(height, width, segRGB, sceneRGB)

    # drawing bounding box
    sceneRGB = drawBB(sceneRGB)

    # time.sleep(1)


def runDataCollect(client, vehicle_name):
    global droneName 
    droneName = str(vehicle_name)
    # set target color in segmentation
    # setBlack = client.simSetSegmentationObjectID("[\w]*", 0, True); # set all other objects to black
    # success = client.simSetSegmentationObjectID('.*?Brian_Dummy.*?', 255, True) # set Brian white

    # take images
    # ImageRequest(name, image_type, pixel_as_float, compress)
    responsesFront = client.simGetImages(
        [airsim.ImageRequest("front", airsim.ImageType.Segmentation, False, False),
        airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)],
        vehicle_name)

    responsesRight = client.simGetImages(
        [airsim.ImageRequest("right", airsim.ImageType.Segmentation, False, False),
        airsim.ImageRequest("right", airsim.ImageType.Scene, False, False)],
        vehicle_name)

    processImageResponses(responsesFront)
    processImageResponses(responsesRight)
    # print("Wolf "+ str(vehicle_name) +": Collected Data!!!")

