import airsim
import numpy as np
import airsim
import math
from HelperFunctions import clusterHelper
import os
import cv2
import time
import pandas as pd
import matplotlib.pyplot as plt

def getInfrared(client, vehicleName):
    # print(vehicleName)
    responses = client.simGetImages([
        airsim.ImageRequest("front-center", airsim.ImageType.Infrared, False, False), 
        airsim.ImageRequest("front-center", airsim.ImageType.Scene, False, False)], vehicle_name = vehicleName)
    return responses

def getScene(client, vehicleName):
    # print(vehicleName)
    responses = client.simGetImages([
        airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)], vehicle_name = vehicleName)
    # ,
    #    airsim.ImageRequest("right", airsim.ImageType.Scene, False, False
    return responses

def getResponse(client, vehicleName, cameraName):
    responses = client.simGetImages([
        airsim.ImageRequest(cameraName, airsim.ImageType.Scene, False, False)], vehicle_name = vehicleName)
    return responses

def getSegInfo(responses):
    responseSeg = responses[0]

    height = responses[0].height
    width = responses[0].width

    segArr = np.fromstring(responseSeg.image_data_uint8, dtype=np.uint8)
    segRGB = segArr.reshape(height, width, 3)

    return height, width, segRGB

def getSceneInfo(responses):
    responseScene = responses[1]

    height = responses[1].height
    width = responses[1].width

    sceneArr = np.fromstring(responseScene.image_data_uint8, dtype=np.uint8)
    sceneRGB = sceneArr.reshape(height, width, 3)

    return height, width, sceneRGB

def getHeightWidthArr(responses, responseIndex):
    responseSeg = responses[responseIndex]

    height = responses[responseIndex].height
    width = responses[responseIndex].width

    segArr = np.fromstring(responseSeg.image_data_uint8, dtype=np.uint8)
    segRGB = segArr.reshape(height, width, 3)

    return height, width, segRGB

def getDroneGPS(vehicleName, client):
    gps_data = client.getGpsData(gps_name = "", vehicle_name = vehicleName)
    gps_al = gps_data.gnss.geo_point.altitude
    gps_lat = gps_data.gnss.geo_point.latitude
    gps_lon = gps_data.gnss.geo_point.longitude
    return gps_al, gps_lat, gps_lon

def midpoint(p1, p2):
    return (p1[0]+p2[0])/2, (p1[1]+p2[1])/2

def gpsDistance(lon1, lat1, lon2, lat2):
    lonR1 = degreeToRadians(lon1)
    latR1 = degreeToRadians(lat1)
    lonR2 = degreeToRadians(lon2)
    latR2 = degreeToRadians(lat2)
    nauticalMiles = 3440.1 * math.acos((math.sin(latR1)*math.sin(latR2)) + math.cos(latR1)*math.cos(latR2) * math.cos(lonR1-lonR2))
    meters = nauticalMiles*1852
    return meters

def degreeToRadians(degree):
    return 3.14*degree/180

def getCentroids(clusters, client, vehicleName, height, width, gps, runtime, segRGB):
    heightArr = []
    widthArr = []
    centroids = []

    # constants needed for geospatial conversion
    earth_rad = 6378000
    pi = math.pi

    # bounding box coorner variables
    y0 = None
    y1 = None
    x0 = None
    x1 = None

    # adjust expected width depending on target distance to center of image
    # target_width = 0.4
    # target_depth = 0.4
    # target_height = 1.3

    target_width = 0.5

    # get drone gps info for geospatial conversion
    # gps_al, gps_lat, gps_lon = getDroneGPS(vehicleName, client)

    print("DELAY: "+str(time.time()-runtime))

    gps_al = gps[0]
    gps_lat = gps[1]
    gps_lon = gps[2]

    ruh_m = np.zeros((500,500,3), np.uint8)
    
    BBox = (-.001, .001, -.001, .001)
    fig, ax = plt.subplots(figsize = (8,7))

    for cluster in clusters:
        for coord in cluster:
            if (coord[0] not in heightArr):
                heightArr.append(coord[0])

            if (coord[1] not in widthArr):
                widthArr.append(coord[1])

        # calculate centroids for current cluster
        if (len(heightArr) != 0):
            # calculate y centroids
            y0 = min(heightArr) # bottom right y
            y1 = max(heightArr) # top left y

            # calculate x centroids
            x0 = min(widthArr) # top left x
            x1 = max(widthArr) # bottom right x

        # calculate x and y of centroid
        xC = ((x1 - x0) / 2) + x0
        yC = ((y1 - y0) / 2) + y0
        centroid = [xC, yC]

        # get bbox dimensions
        bbw = x1 - x0 # width
        bbh = y1 - y0 # height

        if(x0 == 0 or x1 == 0 or x0 == width-1 or x1 >= width-1 or y0 == 0 or y1 == 0 or y0 >= height-1 or y1 == height-1):
            continue

        print("WIDTH!!!"+str(bbw))

        # calculate target distance to center of image
        center_x, center_y = int(width/2), int(height/2)
        center = [center_x, center_y]
        centroid_comp_x = [xC, center_y]
        centroid_comp_y = [center_x, yC]
        center_to_centroid_dist  = clusterHelper.dist(center, centroid)
        center_to_centroid_dist_x  = clusterHelper.dist(center, centroid_comp_x)
        center_to_centroid_dist_y  = clusterHelper.dist(center, centroid_comp_y)

        # signed center to centroid pixel distance
        dist_x  = xC - center_x
        dist_y  = center_y - yC

        # convert pixel distance to meter distance
        # Ratio: known_target_width/bounding_box_width
        # Meter_distance = pixel_distance*meter_known_target_width/pixel_bounding_box_width
        if bbw != 0:
            ratio = target_width/bbw
        else:
            ratio = target_width
        meters_x = dist_x*ratio
        meters_y = dist_y*ratio

        # new latitude and longitude
        lat = gps_lat + (meters_y / earth_rad) * (180/pi)
        lon = gps_lon + (meters_x / earth_rad) * (180/pi)
        #lon = gps_lon + (meters_x / earth_rad) * (180/pi) * (180 / pi) / math.cos(gps_lat * pi/180)

        # add centroid to list
        centroids.append((lon, lat))

        # ax.scatter(lon, lat, zorder=1, alpha= 1, c='#ff43a4', s=10)
        x_values = [float(-2.6949458421474215e-05), float(gps_lon), lon]
        y_values = [float(0.00031441032653474997), float(gps_lat), lat]
        ax.plot(x_values, y_values, 'bo', linestyle="--")

        ax.text(float(gps_lon), float(gps_lat), "droneGPS", color='red', fontsize = 9)
        ax.text(float(-2.6949458421474215e-05), float(0.00031441032653474997), "Brian", color='red', fontsize = 9)
        ax.text(lon, lat, "GPSestimate", color='red', fontsize = 9)

        trueLoc = [float(-2.6949458421474215e-05), float(0.00031441032653474997)]
        estimate = [lon, lat]

        midLon, midLat = midpoint(trueLoc, estimate)

        ax.text(midLon, midLat, str(round(gpsDistance(float(-2.6949458421474215e-05), float(0.00031441032653474997), lon, lat), 2))+"m", color='yellow', fontsize = 9)

    # ax.scatter(float(gps_lon), float(gps_lat), zorder=1, alpha= 1, c='#ffffff', s=10)
    # ax.scatter(float(-2.6949458421474215e-05), float(0.00031441032653474997), zorder=1, alpha= 1, c='#0048ba', s=10)

        ax.set_title('Plotting Spatial Data on Map')
        ax.set_xlim(BBox[0],BBox[1])
        ax.set_ylim(BBox[2],BBox[3])
        ax.imshow(ruh_m, zorder=0, extent = BBox, aspect= 'equal')

        k=0

        cwd = os.getcwd()
        cwd = os.path.join(cwd[:cwd.index("DroneSwarm")],'DroneSwarm')
        plotPath=os.path.join(str(cwd),str(k)+'estimateGraph.png')

        while os.path.exists(os.path.join(str(cwd),str(k)+'esimatePic.png')):
            k+=1

        plotPicPath=os.path.join(str(cwd),str(k)+'esimatePic.png')


        fig.savefig(os.path.join(str(cwd),str(k)+'estimateGraph.png'))
        airsim.write_png(plotPicPath, segRGB)

    return centroids

def getSearchCircles(intersectGroups, avgCentroids, r):
    searchRadii = []

    for (group, avg) in zip(intersectGroups, avgCentroids):
        furthest = clusterHelper.furthestCentroid(group, avg[0], avg[1])
        rSearch = clusterHelper.distanceForm(furthest, avg[0], avg[1]) + r
        searchRadii.append(rSearch)

    return searchRadii

def getSceneImages(responses, folderName):
    height, width, sceneRGB = getSceneInfo(responses)

    cwd = os.getcwd()
    cwd = os.path.join(cwd[:cwd.index("DroneSwarm")],'DroneSwarm')
    savePath=os.path.join(str(cwd),folderName)

    isExist=os.path.exists(savePath)

    if not isExist:
        os.makedirs(savePath)

    k=0
    while os.path.exists(savePath + "/" + ('%s' % k)+folderName+"Scene.jpg"):
        k+=1

    airsim.write_png(savePath + "/" + ('%s' % k)+folderName+"Scene.jpg", sceneRGB)

def getInfraImages(responses, folderName):
    height, width, segRGB = getSegInfo(responses)

    cwd = os.getcwd()
    cwd = os.path.join(cwd[:cwd.index("DroneSwarm")],'DroneSwarm')
    savePath=os.path.join(str(cwd),folderName)

    isExist=os.path.exists(savePath)

    if not isExist:
        os.makedirs(savePath)

    k=0
    while os.path.exists(savePath + "/" + ('%s' % k)+folderName+"Infra.jpg"):
        k+=1

    airsim.write_png(savePath + "/" + ('%s' % k)+folderName+"Infra.jpg", segRGB)

def getInfraredGPSImages(responses, folderName, filteredCentroidsGPS, clusters):
    height, width, segRGB = getSegInfo(responses)
    height, width, sceneRGB = getSceneInfo(responses)

    cwd = os.getcwd()
    cwd = os.path.join(cwd[:cwd.index("DroneSwarm")],'DroneSwarm')
    savePath=os.path.join(str(cwd),folderName)

    isExist=os.path.exists(savePath)

    if not isExist:
        os.makedirs(savePath)

    k=0
    while os.path.exists(savePath + "/" + ('%s' % k)+folderName+"infra.jpg"):
        k+=1

    airsim.write_png(savePath + "/" + ('%s' % k)+folderName+"infra.jpg", segRGB)
    airsim.write_png(savePath + "/" + ('%s' % k)+folderName+"scene.jpg", sceneRGB)

    clusters1 = [clusters[0]] # hilight first cluster green

    drawBB(height, width, sceneRGB, savePath + "/" + ('%s' % k)+folderName+"scene.jpg", clusters, (255, 0, 0))

    drawBB(height, width, sceneRGB, savePath + "/" + ('%s' % k)+folderName+"scene.jpg", clusters1, (0, 255, 0))

    f = open(savePath + "/" + ('%s' % k)+folderName+".txt", "w")
    for centroid in filteredCentroidsGPS:
        f.write("\n\tOverseer Estimate:"+ str(centroid[0]) + " lat, " + str(centroid[1]) + " lon")
    f.close()

def drawBB(height, width, sceneRGB, path, clusters, color):
    heightArr = []
    widthArr= []

    hArr = []
    wArr = []

    clusterLen = len(clusters)

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
        sceneRGB = cv2.rectangle(sceneRGB, start_point, end_point, color, 1)

        hArr=heightArr
        wArr=widthArr

        # reset height and width arrays
        heightArr = []
        widthArr = []
        
        # print("Number of clusters: ", len(clusters))
        # empty cluster list
        clusters = []

    if(clusterLen!=0):
        airsim.write_png(path, sceneRGB)

    pixCount = 0

    return sceneRGB