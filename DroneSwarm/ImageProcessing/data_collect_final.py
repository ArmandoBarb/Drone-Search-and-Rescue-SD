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
from numpy import savetxt
BATCH_NAME = configDrones.BATCH_NAME

warnings.filterwarnings("ignore", category=DeprecationWarning) 

# import for making bounding boxes
#import math
#import matplotlib.pyplot as plt
#import pandas as pd

# heightArr = []
# widthArr = []
blue = (255, 0, 0)  # color of bounding box
thickness = 1       # bounding box thickness
# pixCount = 0        # number of pixels within bounding box #why god
droneName = ''

# replacement for math.dist
# calculates euclidian distance
def dist(P, Q):
    return (sum((pi - qi)**2 for pi, qi in zip(P, Q)))**0.5

# list of pixel clusters (a list of lists)
# clusters = []

def checkSiblingClusterExists(pixelRGB, currentCluster, sceneRGB, clusters):
    if currentCluster==len(clusters):
        return False
    
    for i in range(currentCluster+1, len(clusters)):
        clusterColor = sceneRGB[clusters[i][0][0]][clusters[i][0][1]][0]
        if(pixelRGB==clusterColor):
            return True

    return False


# def pixelClustering(height, width, segRGB, sceneRGB):
#     pixCount = 0
#     clusters = []


#     for i in range(height):
#         for j in range(width):
            
#             if segRGB[i][j][0] <= 255 and \
#                segRGB[i][j][1] <= 255 and \
#                segRGB[i][j][2] <= 255 and \
#                segRGB[i][j][0] >= 254 and \
#                segRGB[i][j][1] >= 254 and \
#                segRGB[i][j][2] >= 254:
#                 pixCount = pixCount + 1

#                 isMatch = False
#                 isClustered = False
#                 pixel = [(i, j)]

#                 # found heat signature pixels
#                 if not clusters:
#                     clusters.append(pixel)
#                 else:
#                     clusterCount = 0
#                     # not empty case
#                     for cluster in clusters:
#                         sibilingExists = False
#                         for coord in cluster:
#                             # properly place the cluster among appropriate color
#                             imgPixel = segRGB[i][j][0]
#                             clusterPixel = segRGB[coord[0]][coord[1]][0]
#                             if imgPixel != clusterPixel:
#                                 break
#                             else:
#                                 isMatch = True
                            
#                             # top = j>0 and j<height/3  and dist([i, j], [coord[0], coord[1]]) < 20 
#                             # mid = j>height/3 and j<2*height/3 and dist([i, j], [coord[0], coord[1]]) < 60
#                             # bottom =  j>2*height/3 and j<height and dist([i, j], [coord[0], coord[1]]) < 100

#                             # if (top or mid or bottom):
                        

#                             # # otherwise threshold and sort the coordinate
#                             if (dist([i, j], [coord[0], coord[1]]) < 100):
#                             #     # print("in dist([i, j], [coord[0], coord[1]]) < 40")
#                                 # print("After dist([i, j], [coord[0], coord[1]]) < 40")
#                                 # print("Cluster size: ", len(cluster))
#                                 # print("Cluster list size: ", len(clusters))
#                                 # print("RGB = ", [segRGB[i][j][0], segRGB[i][j][1], segRGB[i][j][2]])
#                                 cluster.append(pixel[0])
#                                 # print("---------------------------------")
#                                 # print(cluster)
#                                 # print("---------------------------------")
#                                 isClustered = True
#                                 break
#                             else:
#                                 # used for same heat signature for two animals
#                                 # but they are far apart
#                                 sibilingExists = checkSiblingClusterExists(clusterPixel, clusterCount, segRGB, clusters) # dose this have sideeffect
#                                 if not sibilingExists:
#                                     isClustered = True
#                                     clusters.append(pixel)
#                                 break

#                         clusterCount+=1
#                         if isClustered:
#                             break
#                     # no match for non-empty list
#                     if not isMatch:
#                         clusters.append(pixel)

#     return clusters, pixCount

def makeBackgroundPixel(img, pixel, color):
    x = pixel[0]
    y = pixel[1]
    
    img[x][y][0] = color[0]
    img[x][y][1] = color[1]
    img[x][y][2] = color[2]

def isValidPixel(pixel, height, width):
    if 0 <= pixel[0] < height and  0 <= pixel[1] < width:
        return True
    else:
        return False

def isTargetPixel(img, pixel, color, printColor):
    x = pixel[0]
    y = pixel[1]

    if img[x][y][0] == color[0] and \
       img[x][y][1] == color[1] and \
       img[x][y][2] == color[2]:
       return True
    else:
        #if (printColor):
            #print("fail Color: " + str(img[x][y][0]) + " : " + str(img[x][y][1]) + " : " + str(img[x][y][2]) )
        return False

def pixelClustering(height, width, segRGB):
    img = segRGB.copy()
    clusters = []
    backgroundColor = np.array([0, 0, 0])
    targetColor = np.array([255, 255, 255])

    for x1 in range(height):
        for y1 in range(width):
            
            # check for white pixels
            if isTargetPixel(img, (x1, y1), targetColor, False):
                cluster = []
                # mark as background
                makeBackgroundPixel(img, (x1, y1), backgroundColor)
                cluster.append((x1, y1))

                i = 0
                
                # check adjacency
                while (len(cluster) > i):
                    currentPixel = cluster[i]
                    pixCount = 0
                    for x in range(-1, 2):
                        for y in range(-1, 2):
                            # check center
                            if x == 0 and y == 0: continue;
                            
            
                            testPixel = (x + currentPixel[0], y + currentPixel[1])
                            # check out of bounds
                            if not isValidPixel(testPixel, height, width):
                                #print("Failed at isValidPixel:" + str(testPixel)) 
                                continue;
                            if not isTargetPixel(img, testPixel, targetColor, True): 
                                #print("Failed at isTargetPixel:" + str(testPixel)) 
                                continue;


                            makeBackgroundPixel(img, testPixel, backgroundColor)
                            cluster.append(testPixel)
                            pixCount += 1
                    
                    #print("length of cluster: " + str(len(cluster)) + " index: " + str(i) + " pixCount: " + str(pixCount))
                    i += 1
                clusters.append(cluster)

    return clusters

def deNormalize(normalizedValue, newRange):
    newValue = int(normalizedValue * newRange)
    if (newValue >= newRange):
        return (newRange - 1)
    else:
        return newValue
    

def drawBB(height, width, sceneRGB, sceneRGB2, clusters, segRGB):
    heightArr = []
    widthArr = []

    hArr = []
    wArr = []

    dataDir='/home/testuser/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD-1/DroneSwarm/DataCollection'

    bbDir = dataDir + '/bb'
    imgDir = dataDir + '/images'
    labelDir = dataDir + '/labels'
    emptyGtLabelDir = dataDir + '/emptyLabels'
    emptyGtImgDir = dataDir + '/emptyImages'
    count = 0
    hasValidClusters = False 

    j=0
    while os.path.exists(os.path.join(str(labelDir), droneName + "wolf" +str(j) +BATCH_NAME+ "imgScene.txt")):
        j+=1

    b=0
    while os.path.exists(os.path.join(str(emptyGtLabelDir), droneName + "wolf" +str(b) +BATCH_NAME+ "_EMPTY_"+"imgScene.txt")):
        b+=1

    clusterLen = len(clusters)

    if(clusterLen!=0):
        f = open(os.path.join(str(labelDir), droneName + "wolf" +str(j) +BATCH_NAME+ "imgScene.txt"), 'w')
    else:
        f = open(os.path.join(str(emptyGtLabelDir), droneName + "wolf" +str(j) +BATCH_NAME+ "_EMPTY_"+"imgScene.txt"), 'w')
        f.write('')
        f.close()
        emptyGtPath = os.path.join(str(emptyGtImgDir), droneName + "wolf" +str(j) +BATCH_NAME+ "_EMPTY_" +"imgScene.png")
        airsim.write_png(os.path.normpath(emptyGtPath), sceneRGB)
        return sceneRGB

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
        #sceneRGB = cv2.rectangle(sceneRGB, start_point, end_point, blue, thickness)
        hArr=heightArr
        wArr=widthArr

        # save off label
        if ((len(heightArr) != 0) and (len(cluster) >= 30)):
            # calculate bb height
            bbh = y1 - y0

            # calculate x center and y center of bb
            yC = ((y1 - y0) / 2) + y0
            xC = ((x1 - x0) / 2) + x0
            #xC = ((x1 - (x0-math.floor(bbw*.10))) / 2) + (x0-math.floor(bbw*.10))

            # normalize bbox dimensions and centroids
            normxC = xC/width   # divide by width
            normyC = yC/height  # divide by height
            normW = bbw/width   # divide by width
            normH = bbh/height  # divide by height
            #f.write("0 " + str(normxC) + " " + str(normyC) + " " + str(normW*1.29) + " " + str(normH)+"\n")
            f.write("0 " + str(normxC) + " " + str(normyC) + " " + str(normW) + " " + str(normH)+"\n")
            start_point = (deNormalize((normxC - normW/2), width), deNormalize((normyC - normH/2), height))
            end_point = (deNormalize((normxC + normW/2), width), deNormalize((normyC + normH/2), height))
            sceneRGB = cv2.rectangle(sceneRGB, start_point, end_point, blue, thickness)
            hasValidClusters = True

        #saveLabel(height, width, heightArr, widthArr, labelDir, j, f, pixCount)
        
        # reset height and width arrays
        heightArr = []
        widthArr = []
        
        # print("Number of clusters: ", len(clusters))
        # empty cluster list
        clusters = []

    if not hasValidClusters:
        f.close()
        os.remove(os.path.normpath(os.path.join(str(labelDir), droneName + "wolf" +str(j) +BATCH_NAME+ "imgScene.txt")))
        return sceneRGB

    if(clusterLen!=0):
        f.close()

    if(clusterLen!=0):
        saveImgs(sceneRGB2, sceneRGB, bbDir, imgDir, segRGB, j)

    return sceneRGB

def saveLabel(height, width, heightArr, widthArr, labelDir, j, f, pixCount):
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

        # normalize bbox dimensions and centroids
        normxC = xC/width   # divide by width
        normyC = yC/height  # divide by height
        normW = bbw/width   # divide by width
        normH = bbh/height  # divide by height
        # print ("0" + str(props))


        # write yolo gt to text file
        f.write("0 " + str(normxC) + " " + str(normyC) + " " + str(normW) + " " + str(normH)+"\n")

def saveImgs(sceneRGB2, sceneRGB, bbDir, imgDir, segRGB, j):
        print('SAVE PATH', imgDir)
        sceneSavePathOrig = os.path.join(str(imgDir), droneName + "wolf" +str(j) +BATCH_NAME+ "imgScene.png")
        sceneSavePathBB = os.path.join(str(bbDir), droneName + "wolf" +str(j) +BATCH_NAME+ "imgScene.png")
        #sceneSavePathCSV = os.path.join(str(bbDir), droneName + "wolf" +str(j) +BATCH_NAME+ "imgScene.csv")
        
        airsim.write_png(os.path.normpath(sceneSavePathOrig), sceneRGB2)
        airsim.write_png(os.path.normpath(sceneSavePathBB), sceneRGB)
        # (w, h, s) = segRGB.shape
        # new_img = np.zeros((h, w))
        # for i in range(h):
        #     for j in range(w):
        #         if segRGB[i][j][0] == 0:
        #             new_img[i, j] = 0
        #         else:
        #             new_img[i, j] = 1
        # savetxt(os.path.normpath(sceneSavePathCSV), new_img, delimiter=',')

def setupDirectories():
    dataDir='/home/testuser/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD-1/DroneSwarm/DataCollection'
    isExist=os.path.exists(dataDir)

    bbDir = dataDir + '/bb'
    imgDir = dataDir + '/images'
    labelDir = dataDir + '/labels'

    if not isExist:
        # make directory if not already there
        os.makedirs(dataDir)
        print('Created: ' + dataDir)

    # check that directory exists
    isExist = os.path.exists(bbDir)
    if not isExist:
        # make directory if not already there
        os.makedirs(bbDir)
        print('Created: ' + bbDir)

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
    clusters = pixelClustering(height, width, segRGB)

    # dataDir='/home/testuser/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD-1/DroneSwarm/DataCollection'

    # bbDir = dataDir + '/bb'
    # imgDir = dataDir + '/images'
    # labelDir = dataDir + '/labels'

    # j=0
    # while os.path.exists(os.path.join(str(labelDir), droneName + "wolf" +str(j) +BATCH_NAME+ "imgScene.txt")):
    #     j+=1

    # check for no gt
    # if len(clusters) == 0:
    #     saveImgs(sceneRGB2, sceneRGB, bbDir, imgDir, j)
    #     f = open(os.path.join(str(labelDir), droneName + "wolf" +str(j) +BATCH_NAME+ "imgScene.txt"), 'w')
    #     f.write("")
    #     f.close()

    # drawing bounding box (uncomment and get rid of conditional if you want targets)
    sceneRGB = drawBB(height, width, sceneRGB, sceneRGB2, clusters, segRGB)

    time.sleep(1)


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
    #print("Wolf "+ str(vehicle_name) +": Collected Data!!!")