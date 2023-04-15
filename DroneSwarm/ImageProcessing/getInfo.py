import airsim
import numpy as np
import airsim
import math
from HelperFunctions import clusterHelper
import os
import cv2

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

def getCentroids(clusters, client, vehicleName, height, width):
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
    target_width = 0.4
    target_depth = 0.4
    target_height = 1.3

    # get drone gps info for geospatial conversion
    gps_al, gps_lat, gps_lon = getDroneGPS(vehicleName, client)

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

    return centroids

def getSearchCircles(intersectGroups, avgCentroids, r):
    searchRadii = []

    for (group, avg) in zip(intersectGroups, avgCentroids):
        furthest = clusterHelper.furthestCentroid(group, avg[0], avg[1])
        rSearch = clusterHelper.distanceForm(furthest, avg[0], avg[1]) + r
        searchRadii.append(rSearch)

    return searchRadii

# def xyConvert(lat, lon):
#     λ = 0
#     φ_zero = 0
#     φ = 0
#     r = 0

#     x = r*(λ*math.cos(φ_zero))
#     y = r*φ
#     return (x, y)

def getDetectionMap(wolfPos, predPos):
    path = '/home/testuser/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD/DroneSwarm/detectionMaps/' + 'detections' + '.png' 
    positionGT = (0.00031441032653474997, -2.6949458421474215e-05)
    FUDGE_FACTOR = 1000000

    if os.path.exists(path):
        detectMap = cv2.imread(path)
    else:
        detectionMap = np.zeros((2000, 2000, 3))
        cv2.imwrite(path, detectionMap)
        detectMap = cv2.imread(path)

    # # make x and y conversions happen
    # wolfPos = xyConvert(wolfPos[0], wolfLat[1])
    # predPos = xyConvert(predPos[0], predPos[1])
    # positionGT = xyConvert(positionGT[0], positionGT[1])

    print("wolf_x " + str(int(wolfPos[0]*FUDGE_FACTOR)) + ", wolf_y " + str(int(wolfPos[1]*FUDGE_FACTOR)))
    # print("wolf_x " + str(int(wolfPos[0]*FUDGE_FACTOR)) + ", wolf_y " + str(int(wolfPos[1]*FUDGE_FACTOR)))
    # print("wolf_x " + str(int(wolfPos[0]*FUDGE_FACTOR)) + ", wolf_y " + str(int(wolfPos[1]*FUDGE_FACTOR)))

    # draw line between wolf and prediction
    cv2.line(detectMap, (int(wolfPos[0]*FUDGE_FACTOR), abs(int(wolfPos[1]*FUDGE_FACTOR))), (int(predPos[0]*FUDGE_FACTOR), abs(int(predPos[1]*FUDGE_FACTOR))), (255, 255, 0), 2) 

    # draw vehicle postion
    cv2.circle(detectMap, (int(wolfPos[0]*FUDGE_FACTOR), abs(int(wolfPos[1]*FUDGE_FACTOR))), 2, (255, 0, 0), 2)

    # draw predicted gps postion for wolf
    cv2.circle(detectMap, (int(predPos[0]*FUDGE_FACTOR), abs(int(predPos[1]*FUDGE_FACTOR))), 2, (0, 0, 255), 2)

    # draw ground truth postion
    cv2.circle(detectMap, (int(positionGT[0]*FUDGE_FACTOR + 2000/2), abs(int(positionGT[1]*FUDGE_FACTOR + 2000/2))), 2, (0, 255, 0), 2)

    cv2.imwrite(path, detectMap)
