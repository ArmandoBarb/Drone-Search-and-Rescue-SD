import airsim
import os
import numpy as np
import cv2
import pprint
import time
from sklearn import preprocessing
import shutil
import math
import warnings
import Constants.configDrones as configDrones

LOCAL_IP = configDrones.LOCAL_IP
vehicle_name = None
client = None

warnings.filterwarnings("ignore", category=DeprecationWarning)

heightArr = []
widthArr = []
blue = (255, 0, 0)  # color of bounding box
thickness = 1       # bounding box thickness
pixCount = 0        # number of pixels within bounding box
centroidCount = 0   # number of centers
centroids = []      # list of all centroids in given frame
avgCentroidsX = 0
avgCentroidsY = 0
r = 50
history = []        # list of previously encountered waypoints
intersect = []      # list containing groups of intersecting circles
avgCentroids = []   # holds the average centroid per circle group Convert to gps and return list 
searchRadii = []    # list to hold search circle radii per group  Convert to gps and return list

# replacement for math.dist
# calculates euclidian distance
# def dist(P, Q):
#     return (sum((pi - qi)**2 for pi, qi in zip(P, Q)))**0.5

# list of pixel clusters (a list of lists)
clusters = []

# def checkSiblingClusterExists(pixelRGB, currentCluster, sceneRGB):
#     if currentCluster==len(clusters):
#         return False

#     for i in range(currentCluster+1, len(clusters)):
#         clusterColor = segRGB[clusters[i][0][0]][clusters[i][0][1]][0]
#         if(pixelRGB==clusterColor):
#             return True

#     return False


def pixelClustering(height, width, segRGB, sceneRGB):
    global pixCount
    global clusters

    for i in range(height):
        for j in range(width):
            try:
                if segRGB[i][j][0] <= 255 and \
                segRGB[i][j][1] <= 255 and \
                segRGB[i][j][2] <= 255 and \
                segRGB[i][j][0] >= 200 and \
                segRGB[i][j][1] >= 200 and \
                segRGB[i][j][2] >= 200:
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
                                if dist([i, j], [coord[0], coord[1]]) < 40:
                                    cluster.append(pixel[0])
                                    isClustered = True
                                    break
                                else:
                                    # used for same heat signature for two animals
                                    # but they are far apart
                                    sibilingExists = checkSiblingClusterExists(clusterPixel, clusterCount, sceneRGB)
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
            except:
                print("error seg RGB")
                print(type(segRGB[i][j][0]))
                print(segRGB[i][j][0])

def drawBB(sceneRGB):
    global heightArr
    global widthArr
    global clusters
    global centroidCount
    global centroids
    global avgCentroidsX
    global avgCentroidsY

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
        centroid = (xC, yC)

        # add centroid to list
        centroids.append(centroid)
        centroidCount += 1

        # draw bounding box
        bbw = x1 - x0 # width
        start_point = (x0-math.floor(bbw*.10), y1) # top left corner (corrected offset)
        end_point = (x1, y0)                       # bottom right corner (corrected offset)
        sceneRGB = cv2.rectangle(sceneRGB, start_point, end_point, blue, thickness)

        # reset height and width arrays
        heightArr = []
        widthArr = []

        print("Number of clusters: ", len(clusters))

        # empty cluster list
        clusters = []

    return sceneRGB

# def distanceForm(centroid, avgCentroidsX, avgCentroidsY):
#     distance = ((centroid[0] - avgCentroidsX)**2 + \
#                 (centroid[1] - avgCentroidsY)**2)**0.5

#     return distance

# function to calculate the far centroid
# def furthestCentroid(sceneRGB, centroids, avgCentroidsX, avgCentroidsY):
#     far = 0
#     furthestCentroid = (0, 0)
#     for centroid in centroids:
#         # calculate distance between center and one of the cluster centroids
#         distance = distanceForm(centroid, avgCentroidsX, avgCentroidsY)
#
#         # compare the distance to find the furthest centroid
#         if distance >= far:
#             furthestCentroid = centroid
#             far = distance
#
#     # mark the furthest centroid
#     cv2.circle(sceneRGB, (int(furthestCentroid[0]), int(furthestCentroid[1])), \
#                radius=1, color=(164, 0, 255), thickness=1, lineType=8, \
#                shift=0)
#
#     return furthestCentroid

# def furthestCentroid(centroids, avgCentroidsX, avgCentroidsY):
#     far = 0
#     furthestCentroid = (0, 0)
#     for centroid in centroids:
#         # calculate distance between center and one of the cluster centroids
#         distance = distanceForm(centroid, avgCentroidsX, avgCentroidsY)

#         # compare the distance to find the furthest centroid
#         if distance >= far:
#             furthestCentroid = centroid
#             far = distance

#     return furthestCentroid

def drawOverlap(sceneRGB, r):
    global centroids

    for centroid in centroids:
        cv2.circle(sceneRGB, (int(centroid[0]), int(centroid[1])), \
                   radius=r, color=(0, 0, 255), thickness=1, lineType=8, \
                   shift=0)

    return sceneRGB

def circleGroups(r):
    global intersect
    global centroids
    global avgCentroids
    inGroup = []
    elementsPerGroup = []
    i = 0

    # there's a possible speed up to this loop (TODO: find an optimized approach)
    for centroid in centroids:
        # create circle group
        if len(intersect) == 0 or centroid not in inGroup:
            intersect.append([centroid])
            inGroup.append(centroid)    # keep track of what's in a group
            avgCentroids.append(centroid)
            elementsPerGroup.append(1)  # track the number of circles in group
        else:
            continue

        for centroidPrime in centroids:
            # skip the same element or already grouped elements
            if centroid == centroidPrime or centroidPrime in inGroup:
                continue

            # calculate distance between centers
            d = distanceForm(centroid, centroidPrime[0], centroidPrime[1])

            # check for touching, intersecting circles, and when
            # the center of one lies on the center of another
            if d == 2*r or d == r - r or (d < 2*r and d > r - r) \
               or d == r:
               # add circle to the group of the
               # centroid we were comparing it to
               intersect[i].append(centroidPrime)
               inGroup.append(centroidPrime)

               # start taking an average for the ith group
               avgCentroids[i] = list(avgCentroids[i])
               (avgCentroids[i])[0] += centroidPrime[0]
               (avgCentroids[i])[1] += centroidPrime[1]
               avgCentroids[i] = tuple(avgCentroids[i])

               # update the number of elements in current group
               elementsPerGroup[i] += 1

        # update the group counter
        i += 1

    print("Centroids len: ", len(avgCentroids))
    # we are calculating the number of groups correctly
    # but if two groups are present, a search circle for one gets drawn
    # need to fix this issue
    print("Number of circle groups ", len(intersect))

    # finish the the averaging by dividing each group's
    # average centroid by the number of circles in that group
    for j in range(len(avgCentroids)):
        # divide x and y component by number of circle centers
        avgCentroids[j] = list(avgCentroids[j])
        (avgCentroids[j])[0] /= elementsPerGroup[j]
        (avgCentroids[j])[1] /= elementsPerGroup[j]
        avgCentroids[j] = tuple(avgCentroids[j])

# debug function for displaying and seeing if our circles are correct
def drawCircles(sceneRGB, intersect, avgCentroids, searchRadii, r):
    for (group, avg, radii) in zip(intersect, avgCentroids, searchRadii):
        # plot the center of a particular group
        cv2.circle(sceneRGB, (int(avg[0]), int(avg[1])), \
                   radius=1, color=(0, 255, 0), thickness=1, lineType=8, \
                   shift=0)

        # plot all the circles in current group
        for centroid in group:
            cv2.circle(sceneRGB, (int(centroid[0]), int(centroid[1])), \
                       radius=int(r), color=(255, 0, 0), thickness=1, lineType=8, \
                       shift=0)

        # plot the overall search circle calculate for the group
        cv2.circle(sceneRGB, (int(avg[0]), int(avg[1])), \
                   radius=int(radii), color=(0, 0, 255), thickness=1, lineType=8, \
                   shift=0)

        return sceneRGB

def searchCircleCal(intersect, avgCentroids, r):
    global searchRadii

    for (group, avg) in zip(intersect, avgCentroids):
        furthest = furthestCentroid(group, avg[0], avg[1])
        rSearch = distanceForm(furthest, avg[0], avg[1]) + r
        searchRadii.append(rSearch)

# directory to store pictures
coorDir = r'C:\Users\marii\source\repos\AirSim\PythonClient\custom_scripts\coordinate_extraction'

# check that directory exists
isExist = os.path.exists(coorDir)
if not isExist:
    os.makedirs(coorDir) # make directory if not already there
else:
    shutil.rmtree(coorDir)
    os.makedirs(coorDir) # make directory if not already there

# set up client object to access multirotor drone
# client = airsim.MultirotorClient(LOCAL_IP)  # home ipv4
#client = airsim.MultirotorClient("10.32.33.209")    # school ipv4

# Note: API Control must be off if you want to manually fly drone
# client.enableApiControl(False, vehicle_name)
# client.armDisarm(True, vehicle_name)

def applyInfrared(client):
    # set target color in segmentation
    client.simSetSegmentationObjectID("[\w]*", 0, True);
    client.simSetSegmentationObjectID('.*?FoxMasterAi.*?', 255, True);  # fox
    client.simSetSegmentationObjectID('.*?StagMasterAi.*?', 230, True); # stag
    client.simSetSegmentationObjectID('.*?DoeMasterAi.*?', 200, True);  # doe

    # for debugging
    client.simSetSegmentationObjectID('.*?BrianMasterAi.*?', 255, True);# brian


def waypointDetect(i, droneName, overseerClient):
    global client
    client = overseerClient
    print("Drone:", droneName, " Doing waypoint detection")
    global vehicle_name
    vehicle_name = droneName

    gps_data = client.getGpsData(gps_name = "", vehicle_name = vehicle_name)
    print("Got gps data", gps_data)


    responses = client.simGetImages([
        airsim.ImageRequest("front-center", airsim.ImageType.Infrared, False, False), 
        airsim.ImageRequest("front-center", airsim.ImageType.Scene, False, False)], vehicle_name = vehicle_name)

    # ImageRequest(name, image_type, pixel_as_float, compress)
    #request = [airsim.ImageRequest("front-center", airsim.ImageType.Infrared, False, False),
    #     airsim.ImageRequest("front-center", airsim.ImageType.Scene, False, False)]
    # request = airsim.ImageRequest("front-center", airsim.ImageType.Infrared, False, False)

    # responses = client.simGetImages(requests = request, vehicle_name = vehicle_name)
    
    print("Drone:", droneName, " Got responses")

    responseScene = responses[1]
    responseSeg = responses[0]

    height = responses[0].height
    width = responses[0].width

    sceneArr = np.fromstring(responseScene.image_data_uint8, dtype=np.uint8) # bw target before
    segArr = np.fromstring(responseSeg.image_data_uint8, dtype=np.uint8)

    sceneRGB = sceneArr.reshape(height, width, 3) # shape image
    segRGB = segArr.reshape(height, width, 3)

    # cluster animal heat signatures
    pixelClustering(height, width, segRGB, sceneRGB)

    # drawing bounding box
    sceneRGB = drawBB(sceneRGB)

    # finish averaging all the centroids
    if centroidCount != 0:
        # calculate cluster groups
        # and get average centroid for each group
        circleGroups(r)

        # calculate furthest centroid for each circle cluster
        # calculate the overall search area circle for each circle cluster
        # while taking into account that each waypoint circle radius
        # will involve the distance to
        # the far centroid plus the standard radius distance
        searchCircleCal(intersect, avgCentroids, r)

        # for debugging: plot out the circles for each cluster
        # along with their centers and their overall search circles
        sceneRGB = drawCircles(sceneRGB, intersect, avgCentroids, searchRadii, r)

    sceneSavePath = coorDir+ "\\"+str(i)+"SceneBB.png" # Corrected BB offset

    # sceneRGB, has black background and target covered in white
    airsim.write_png(os.path.normpath(sceneSavePath), sceneRGB)

    print('Target Calculations: \n\tTarget area: ', str(pixCount)+" px")

    y0, y1, x0, x1 = None, None, None, None

    if (len(heightArr) != 0):
        # calculate y centroids
        y0 = min(heightArr) # bottom right y
        y1 = max(heightArr) # top left y

        # calculate x centroids
        x0 = min(widthArr) # top left x
        x1 = max(widthArr) # bottom right x


    # only estimates when entire object iis in sight
    if ((len(heightArr) != 0) and (pixCount >= 2) and (not (x0<5 or y0<5 or x1>width-5 or y1>height-5))):
        # calculate x and y of centroid
        xC = ((x1 - x0) / 2) + x0
        yC = ((y1 - y0) / 2) + y0
        centroid = [xC, yC]

        # get bbox dimensions
        bbw = x1 - x0 # width
        bbh = y1 - y0 # height

        print("\tBounding box width: "+ str(bbw))
        print("\tBounding box height: "+ str(bbh))

        # calculate target distance to center of image
        center_x, center_y = int(width/2), int(height/2)
        center = [center_x, center_y]
        centroid_comp_x = [xC, center_y]
        centroid_comp_y = [center_x, yC]
        center_to_centroid_dist  = dist(center, centroid)

        center_to_centroid_dist_x  = dist(center, centroid_comp_x)
        center_to_centroid_dist_y  = dist(center, centroid_comp_y)

        # adjust expected width depending on target distance to center of image
        target_width = 0.4
        target_depth = 0.4
        target_height = 1.3

        # ratio_to_height = target_width/target_height

        # convert pixels to inches
        # calculate focal length using FOV (horizontal FOV supported in settings.json)
        # Vertical FoV can be calculated by the aspect ratio: vertical FoV = image_height / image_width * horizontal FoV

        #foci_distance = (78.5*3)/2 # calculates foci distance constant (target_pixel_width*actual_target_distance/actual_target_width)

        #normalized_center_to_centroid_dist = center_to_centroid_dist_x/center_x
        #print("\tNORMALIZED DIST TO CENTER: "+str(normalized_center_to_centroid_dist))


        #expected_target_width_meters = math.hypot(target_width, target_height*normalized_center_to_centroid_dist)
        #distance_to_target = (foci_distance*expected_target_width_meters)/bbw

        gps_al = gps_data.gnss.geo_point.altitude
        gps_lat = gps_data.gnss.geo_point.latitude
        gps_lon = gps_data.gnss.geo_point.longitude

        #al = gps_al-distance_to_target

        # signed center to centroid pixel distance
        dist_x  = xC-center_x
        dist_y  = center_y-yC

        # convert pixel distance to meter distance
        # Ratio: known_target_width/bounding_box_width
        # Meter_distance = pixel_distance*meter_known_target_width/pixel_bounding_box_width
        ratio = target_width/bbw
        meters_x = dist_x*ratio
        meters_y = dist_y*ratio

        # new latitude and longitude
        earth_rad = 6378000
        pi = math.pi

        lat = gps_lat + (meters_y / earth_rad) * (180/pi)
        lon = gps_lon + (meters_x / earth_rad) * (180/pi) * (180 / pi) / math.cos(gps_lat * pi/180)

        print("\tCenter to centroid distance: "+str(center_to_centroid_dist)+" px")
        print("\tCenter to centroid signed distance_x: "+str(dist_x)+" px, "+str(meters_x)+" m")
        print("\tCenter to centroid signed distance_y: "+str(dist_y)+" px, "+str(meters_y)+" m")

       # print("\tFoci distance: "+str(foci_distance)+" m")
        #print("\tDistance to target: "+str(distance_to_target)+" m")
        print("\tKnown target height: "+str(target_height)+" m")
        print("\tKnown target width: "+str(target_width)+" m")
        #print("\tExpected target width: "+str(expected_target_width_meters)+" m")
        print("\tDrone location: "+str(gps_data.gnss.geo_point.altitude)+" alt, " + str(gps_lat) + " lat, " + str(gps_lon) + " lon")

        print("------------------------------------------------------------------------------------------------------------------")
        print("Results:")

        known_alt = round(1.6411616802215576, 5)
        known_lat = float(-4.685798977012554e-07)
        known_lon = float(5.1650697592176985e-05)

        print("\tEstimated target location: "+str(known_alt)+" alt, " + str(lat) + " lat, " + str(lon) + " lon")
        print("\tActual target location: "+str(known_alt)+" alt, " + str(known_lat) + " lat, " + str(known_lon) + " lon")
        print("\tMargin of Error: "+str(known_alt-known_alt)+" alt, " + str(known_lat-lat) + " lat, " + str(known_lon-lon) + " lon")
        meters_y_er = ((known_lat-lat)*earth_rad)/(180/pi)
        meters_x_er = ((known_lon-lon) * math.cos(gps_lat * pi/180)*earth_rad)/((180/pi) * (180 / pi))
        print("\tMargin of Error (meters): "+str(meters_x_er)+" mx, " + str(meters_y_er) + " my")

        print("------------------------------------------------------------------------------------------------------------------")

        # normalize bbox dimensions and centroids
        props = preprocessing.normalize([np.array([xC, yC, bbw, bbh])])

        # write yolo gt to text file
        with open(coorDir + "\\" + str(i) + "GPSEstimate.txt", 'w') as f:
            f.write("Normalized Coordinates: ")
            f.write("0 " + str(props[0][0]) + " " + str(props[0][1]) + " " + str(props[0][2]) + " " + str(props[0][3]))
            #f.write("\nDistance to Target: "+str(distance_to_target)+" m")
            #f.write("\nExpected Target Width: "+str(expected_target_width_meters)+" m")
            f.write("\nmeters_x: "+str(meters_x)+" m" )
            f.write("\nmeters_y: "+str(meters_y)+" m")
            f.write("\nLat conversion: lat = gps_lat + (meters_y / earth_rad) * (180/pi): ")
            f.write("\ngps_lat: "+str(gps_lat))
            f.write("\nLon conversion: (meters_y / earth_rad): "+str(meters_y / earth_rad))
            f.write("\n(180/pi): "+str(180/pi))
            f.write("\nEstimated Target Location: "+str(round(known_alt, 5))+" alt, " + str("{:.5f}".format(float(lat))) + " lat, " + str(round(lon, 5)) + " lon")
            f.write("\nActual Target Location: "+str(round(14.3292503, 5))+" alt, " + str(round(1.70164796886, 5)) + " lat, " + str(round(59.9999623, 5)) + " lon")
            f.write("\nDrone Location: "+str(round(gps_data.gnss.geo_point.altitude, 5))+" alt, " + str("{:.5f}".format(float(gps_lat)))+ " lat, " + str(round(gps_lon, 5)) + " lon")
            f.write("\n------------------------------------------------------------------------------------------------------------------")
            f.write("Bounding Box Info:")
            f.write("\nx0 (top left): "+str(x0))
            f.write("\ny0 (top left): "+str(y0))
            f.write("\nx1 (bottom right): "+str(x1))
            f.write("\ny1 (bottom right): "+str(y1))
            f.write("\nwidth: "+str(width))
            f.write("\nheight: "+str(height))
            f.close()

    # # reset counters and lists for next frame
    # pixCount = 0
    # centroidCount = 0
    # avgCentroidsX = 0
    # avgCentroidsY = 0
    # centroids = []
    # intersect = []
    # avgCentroids = []
    # searchRadii = []

    print("GPS estimation cycle complete.")
    print("\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n")

    if (len(centroids) != 0):
        return (avgCentroids, searchRadii)
    else:
        return None
