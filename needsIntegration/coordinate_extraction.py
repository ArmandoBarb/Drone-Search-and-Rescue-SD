import airsim
import os
import numpy as np
import cv2
import pprint
import time
from sklearn import preprocessing
import shutil
import math

# import for making bounding boxes
#import math
#import matplotlib.pyplot as plt
#import pandas as pd

heightArr = []
widthArr = []
blue = (255, 0, 0)  # color of bounding box
thickness = 1       # bounding box thickness
pixCount = 0        # number of pixels within bounding box

def blackpixels(height, width, segRGB, sceneRGB):
 # black pixel loop
    global pixCount
    for i in range(height):
        for j in range(width):
            if segRGB[i][j][0] == 255 and segRGB[i][j][1] == 255 and segRGB[i][j][2] == 255:
                sceneRGB[i][j]=(255,255,255)
                sceneRGB[i][j]=(255,255,255)
                sceneRGB[i][j]=(255,255,255)
                pixCount = pixCount + 1
                
                if (i not in heightArr):
                    heightArr.append(i)
                
                if (j not in widthArr):
                    widthArr.append(j)
            else:
                sceneRGB[i][j]=(0,0,0)
                sceneRGB[i][j]=(0,0,0)
                sceneRGB[i][j]=(0,0,0)

    return sceneRGB

# directory to store pictures
coorDir = r'C:\Users\marii\OneDrive\Documents\coordinate_extraction'

vehicle_name = '0'

# check that directory exists
isExist = os.path.exists(coorDir)
if not isExist:
    # make directory if not already there
    os.makedirs(coorDir)
    print('Created: ' + coorDir)
else:
    shutil.rmtree(coorDir)
    # make directory if not already there
    os.makedirs(coorDir)
    print('Created: ' + coorDir)

# set up client object to access multirotor drone
client = airsim.MultirotorClient()

# connect to AirSim simulator
client.confirmConnection()

# Note: API Control must be off if you want to manually fly drone
client.enableApiControl(False, vehicle_name)
client.armDisarm(True, vehicle_name)

# set target color in segmentation
success1 = client.simSetSegmentationObjectID("[\w]*", 0, True);
success2 = client.simSetSegmentationObjectID("Ground[\w]*", 150, True);
success2 = client.simSetSegmentationObjectID("Red[\w]*", 100, True);
success = client.simSetSegmentationObjectID("Cylinder8", 255, True);
# objectId = client.simGetSegmentationObjectID("Cylinder8");
# print('Cylinder Object ID:', str(objectId))


targetRGB = [ 0, 0, 0]
if success == True:
    print('PersonObject Found Value is: ' + str(targetRGB))

i=0;
# image collection loop
#while i<1000:
while i < 100:

    gps_data = client.getGpsData(gps_name = "", vehicle_name = vehicle_name)
    print("GPS DATA TYPE:"+str(type(gps_data)))

    print("GPS DATA:"+str(gps_data))

    print("GPS ALTITUDE:"+str(gps_data.gnss.geo_point.altitude))

    # take images
    # ImageRequest(name, image_type, pixel_as_float, compress)
    responses = client.simGetImages(
        [airsim.ImageRequest("front-center", airsim.ImageType.Infrared, False, False),
        airsim.ImageRequest("front-center", airsim.ImageType.Scene, False, False),
        airsim.ImageRequest("front-center", airsim.ImageType.Infrared, False, False)],
        vehicle_name)

#    responses2 = client.simGetImages(
#    [airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)],
#    vehicle_name)

    responseScene = responses[1]
    responseSeg = responses[0]
#    responseSeg2 = responses2[0] # second segmentation for offset


    height = responses[0].height
    width = responses[0].width

    print('Retrieved scene images: ', len(responses))

    targetArr = np.fromstring(responseScene.image_data_uint8, dtype=np.uint8) # bw target before
    sceneArr = np.copy(targetArr) # bw target after
    sceneArr2 = np.copy(targetArr)   
    segArr = np.fromstring(responseSeg.image_data_uint8, dtype=np.uint8)
#    segArr2 = np.fromstring(responseSeg2.image_data_uint8, dtype=np.uint8)

    # shape image
    targetRGB = targetArr.reshape(height, width, 3)
    sceneRGB = sceneArr.reshape(height, width, 3)
    sceneRGB2 = sceneArr2.reshape(height, width, 3)
    segRGB = segArr.reshape(height, width, 3)
    # segRGB2 = segArr2.reshape(height, width, 3)

    bwRGB = blackpixels(height, width, segRGB, targetRGB)
    # bwRGB2 = blackpixels(height, width, segRGB2, targetRGB2)

    print('Pixel count for target: ', str(pixCount))

    y0, y1, x0, x1 = None, None, None, None

    if (len(heightArr) != 0):
        # calculate y centroids
        y0 = min(heightArr)
        y1 = max(heightArr)

        # calculate x centroids
        x0 = min(widthArr)
        x1 = max(widthArr)


    if ((len(heightArr) != 0) and (pixCount >= 2) and (not (x0<5 or y0<5 or x1>width-5 or y1>height-5))):
        # calculate y centroids
        yC = ((y1 - y0) / 2) + y0

        # calculate x centroids
        xC = ((x1 - x0) / 2) + x0
        print("(xC, yC) = " + str(xC) + " " + str(yC))

        centroid = [xC, yC]

        # drawing bounding box
        start_point = (x0, y1) # top left corner
        end_point = (x1, y0)   # bottom right corner
        sceneRGB = cv2.rectangle(sceneRGB, start_point, end_point, blue, thickness)

        print("top left x: "+ str(x0))
        print("top left y: "+ str(y1))
        print("bottom right x: "+ str(x1))
        print("bottom right y: "+ str(y0))

        # exclude bounding boxes that are too close to the edge of the image
       # if(x0==0 or y1==0 or x1==width or y0==height):
         #   continue

        # get bbox dimensions
        bbw = x1 - x0
        bbh = y1 - y0

        print("BOUND BOX WIDTH: "+ str(bbw))
        print("BOUND BOX HEIGHT: "+ str(bbh))
        print("---------------------------------------------------------")
        
        # calculate target distance to center of image
        center_x, center_y = int(width/2), int(height/2)
        center = [center_x, center_y]
        centroid_comp_x = [xC, center_y]
        centroid_comp_y = [center_x, yC]
        center_to_centroid_dist  = math.dist(center, centroid)

        center_to_centroid_dist_x  = math.dist(center, centroid_comp_x)
        center_to_centroid_dist_y  = math.dist(center, centroid_comp_y)

        # adjust expected width depending on target distance to center of image
        target_width = 2
        target_depth = 2
        target_height = 5

        ratio_to_height = target_width/target_height

        foci_distance = (78.5*3)/2 # calculates foci distance constant (target_pixel_width*actual_target_distance/actual_target_width)

        #max_center_dist  = math.dist(center, [0,0])
        #normalized_center_to_centroid_dist = center_to_centroid_dist/max_center_dist
        normalized_center_to_centroid_dist = center_to_centroid_dist_x/center_x
        #expected_target_width_meters = target_width+(target_height*ratio_to_height)*normalized_center_to_centroid_dist # 2+5*(normalized_center_to_centroid_dist)

        expected_target_width_meters = math.hypot(target_width, target_height*normalized_center_to_centroid_dist)
        #distance_to_target = (foci_distance*2)/bbw
        distance_to_target = (foci_distance*expected_target_width_meters)/bbw


        gps_al = gps_data.gnss.geo_point.altitude
        gps_lat = gps_data.gnss.geo_point.latitude      
        gps_lon = gps_data.gnss.geo_point.longitude

        al = gps_al-distance_to_target

        # signed center to centroid pixel distance
        dist_x  = xC-center_x
        dist_y  = center_y-yC

        # convert pixel distance to meter distance
        meters_x = dist_x*expected_target_width_meters/bbw
        meters_y = dist_y*expected_target_width_meters/bbw

        # new latitude and longitude
        earth_rad = 6378000
        pi = math.pi

        lat = gps_lat + (meters_y / earth_rad) * (180/pi)
        lon = gps_lon + (meters_x / earth_rad) * (180/pi) * (180 / pi) / math.cos(gps_lat * pi/180)

      #  lat = gps_lat + (dist_y / earth_rad) * (180/pi)
      #  lon = gps_lon + (dist_x / earth_rad) * (180/pi) * (180 / pi) / math.cos(gps_lat * pi/180)

        print("CENTER TO CENTROID DISTANCE: "+str(center_to_centroid_dist)+" px")

        print("CENTER TO CENTROID_X SIGNED DISTANCE: "+str(dist_x)+" px")
        print("CENTER TO CENTROID_Y SIGNED DISTANCE: "+str(dist_y)+" px")

        print("CENTER TO CENTROID_X SIGNED DISTANCE: "+str(meters_x)+" M")
        print("CENTER TO CENTROID_Y SIGNED DISTANCE: "+str(meters_y)+" M")

        print("FOCI DISTANCE: "+str(foci_distance)+" M")
        print("DISTANCE TO TARGET: "+str(distance_to_target)+" M")

        print("---------------------------------------------------------")

        print("EXPECTED ACTUAL TO TARGET WIDTH: "+str(expected_target_width_meters)+" M")

        print("meters_x: "+str(meters_x)+" M" )
        print("meters_y: "+str(meters_y)+" M")


        print("ESTIMATED TARGET LOCATION: "+str(al)+" ALT, " + str(lat) + " LAT, " + str(lon) + " LON")
        print("ACTUAL TARGET LOCATION: "+str(14.3292503)+" ALT, " + str(1.70164796886) + " LAT, " + str(59.9999623) + " LON")


        print("CURRENT LOCATION: "+str(gps_data.gnss.geo_point.altitude)+" ALT, " + str(gps_lat) + " LAT, " + str(gps_lon) + " LON")

        print("---------------------------------------------------------")

        # drawing bounding box
        start_point2 = (x0-math.floor(bbw*.10), y1) # top left corner
        end_point2 = (x1, y0)   # bottom right corner
        sceneRGB2 = cv2.rectangle(sceneRGB2, start_point2, end_point2, blue, thickness)

        # normalize bbox dimensions and centroids
        props = preprocessing.normalize([np.array([xC, yC, bbw, bbh])])
        print ("0" + str(props))

        # write yolo gt to text file
        with open(coorDir + "\\" + str(i) + "imgScene.txt", 'w') as f:
            f.write("0 " + str(props[0][0]) + " " + str(props[0][1]) + " " + str(props[0][2]) + " " + str(props[0][3]))
            f.write("\nDISTANCE TO TARGET: "+str(distance_to_target)+" M")
            f.write("\nEXPECTED ACTUAL TO TARGET WIDTH: "+str(expected_target_width_meters)+" M")
            f.write("\nmeters_x: "+str(meters_x)+" M" )
            f.write("\nmeters_y: "+str(meters_y)+" M")
            f.write("\nLAT CALC lat = gps_lat + (meters_y / earth_rad) * (180/pi): ")
            f.write("\ngps_lat: "+str(gps_lat))
            f.write("\n(meters_y / earth_rad): "+str(meters_y / earth_rad))
            f.write("\n(180/pi): "+str(180/pi))
            f.write("\nESTIMATED TARGET LOCATION: "+str(round(al, 5))+" ALT, " + str("{:.5f}".format(float(lat))) + " LAT, " + str(round(lon, 5)) + " LON")
            f.write("\nACTUAL TARGET LOCATION: "+str(round(14.3292503, 5))+" ALT, " + str(round(1.70164796886, 5)) + " LAT, " + str(round(59.9999623, 5)) + " LON")
            f.write("\nCURRENT LOCATION: "+str(round(gps_data.gnss.geo_point.altitude, 5))+" ALT, " + str("{:.5f}".format(float(gps_lat)))+ " LAT, " + str(round(gps_lon, 5)) + " LON")
            f.write("\nx0 (top left): "+str(x0))
            f.write("\ny0 (top left): "+str(y0))
            f.write("\nx1 (bottom right): "+str(x1))
            f.write("\ny1 (bottom right): "+str(y1))
            f.write("\nwidth: "+str(width))
            f.write("\nheight: "+str(height))
            f.close()

        # empty out the arrays
        # heightArr = []
        # widthArr = []
   
        targetSavePath = coorDir+ "\\"+str(i)+"targetSceneBefore.png"
        target2SavePath = coorDir+ "\\"+str(i)+"targetSceneAfter.png"

        sceneSavePath = coorDir+ "\\"+str(i)+"imgScene.png"
        sceneSavePath2 = coorDir+ "\\"+str(i)+"imgSceneCorrected.png"
        segSavePath = coorDir+ "\\"+str(i)+"imgSeg.png"

        # bwRGB, has black background and target covered in white
        airsim.write_png(os.path.normpath(targetSavePath), bwRGB)
#        airsim.write_png(os.path.normpath(target2SavePath), bwRGB2)

        airsim.write_png(os.path.normpath(sceneSavePath), sceneRGB)
        airsim.write_png(os.path.normpath(sceneSavePath2), sceneRGB2)
        airsim.write_png(os.path.normpath(segSavePath), segRGB)
        time.sleep(5)

    
    # reset pixel count
    pixCount = 0

    # empty out the arrays
    heightArr = []
    widthArr = []

    time.sleep(1)
    i+=1

    print("DATA COLLECTION COMPLETE!!!")

print("vertical min = " + str(min(heightArr))) # bottom value
print("vertical max = " + str(max(heightArr))) # top value
print("horizontal min = " + str(min(widthArr))) # leftmost value
print("horizontal max = " + str(max(widthArr))) # rightmost value

