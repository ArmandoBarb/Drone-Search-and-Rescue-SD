import airsim
import numpy as np
import math
from sklearn import preprocessing
import math
import time
from ImageProcessing import getInfo 

def getWolfGPSEstimate(client, responses, vehicle_name, xMin, yMin, xMax, yMax):
    gps_al, gps_lat, gps_lon = getInfo.getDroneGPS(vehicle_name, client)
    responseScene = responses[0]

    x=responseScene.camera_orientation.x_val
    y=responseScene.camera_orientation.y_val
    z=responseScene.camera_orientation.z_val
    w=responseScene.camera_orientation.w_val

    # print('Camera Orientation: ', str(responseScene.camera_orientation))
    
    # print('Camera X: ', str(x))
    # print('Camera Y: ', str(y))
    # print('Camera Z: ', str(z))
    # print('Camera W: ', str(w))
    
    roll  = (180/math.pi)*math.atan2(2*y*w + 2*x*z, 1 - 2*y*y - 2*z*z)
    pitch = (180/math.pi)*math.atan2(2*x*w + 2*y*z, 1 - 2*x*x - 2*z*z)
    yaw0 =  (180/math.pi)*math.asin(2*x*y + 2*z*w)
    yaw = 0

    if(w>.5 and yaw0>0):
        yaw =  90-yaw0
    elif(w<.5 and yaw0>=0):
        yaw =  -90+yaw0
    elif(w>.5 and yaw0<=0):
        yaw = 90-yaw0
    elif(w<.5 and yaw0<0):
        yaw = -90+yaw0

    # print('Camera Roll: ', str(roll))
    # print('Camera Pitch: ', str(pitch))
    # print('Camera Yaw: ', str(yaw))

    y0 = yMin
    y1 = yMax
    x0 = xMin
    x1 = xMax

    # print("\tDrone location: "+str(gps_al)+" alt, " + str(gps_lat) + " lat, " + str(gps_lon) + " lon")

    # only estimates when entire object is in sight
    # TODO: if statement
    # calculate x and y of centroid
    xC = ((x1 - x0) / 2) + x0
    yC = ((y1 - y0) / 2) + y0
    centroid = [xC, yC]

    # get bbox dimensions
    bbw = x1 - x0 # width
    bbh = y1 - y0 # height

    height = responses[0].height
    width = responses[0].width

    if (bbw<4):
        # print("Target too small!!!!!! SKIP")
        return None, None, None

    # print("\tBounding box width: "+ str(bbw))
    # print("\tBounding box height: "+ str(bbh))
    
    # calculate target distance to center of image
    center_x, center_y = int(width/2), int(height/2)
    center = [center_x, center_y]
    centroid_comp_x = [xC, center_y]
    centroid_comp_y = [center_x, yC]
    center_to_centroid_dist  = ((center[0]-centroid[0])**2 + (center[1]-centroid[1])**2)**.5

    center_to_centroid_dist_x  = ((center[0]-centroid_comp_x[0])**2 + (center[1]-centroid_comp_x[1])**2)**.5
    center_to_centroid_dist_y  = ((center[0]-centroid_comp_y[0])**2 + (center[1]-centroid_comp_y[1])**2)**.5

    # adjust expected width depending on target distance to center of image
    target_width = 0.24
    target_depth = 0.24
    target_height = 0.9

    ratio_to_height = target_width/target_height

    # convert pixels to inches
    # calculate focal length using FOV (horizontal FOV supported in settings.json)
    # Vertical FoV can be calculated by the aspect ratio: vertical FoV = image_height / image_width * horizontal FoV

    foci_distance = (78.5*3)/2 # calculates foci distance constant (target_pixel_width*actual_target_distance/actual_target_width) 


    max_dist = ((center[0])**2 + (center[1])**2)**.5
    normalized_center_to_centroid_dist = center_to_centroid_dist_x/max_dist
    # print("\tNORMALIZED DIST TO CENTER: "+str(normalized_center_to_centroid_dist))

    # expected_target_width_meters = math.hypot(target_width, target_height*normalized_center_to_centroid_dist)
    # expected_target_width_meters = (((target_width)**2 + (target_height)**2)**.5) * normalized_center_to_centroid_dist
    
    # extimated_distance_to_brian =  (focal_length_px * known_target_width_meters) / (target_width_px)
    distance_to_target = (foci_distance*(target_width+target_width*normalized_center_to_centroid_dist*.5))/(bbw)

    al = gps_al-distance_to_target

    # # signed center to centroid pixel distance
    # dist_x  = xC-center_x
    # dist_y  = center_y-yC

    meters_x = distance_to_target*math.cos(math.radians(yaw))
    meters_y = distance_to_target*math.sin(math.radians(yaw))

    # new latitude and longitude
    earth_rad = 6378000
    pi = math.pi

    lat = gps_lat + (meters_y / earth_rad) * (180/pi)
    lon = gps_lon + (meters_x / earth_rad) * (180/pi) 

    # print("\tMeters East/West: "+str(meters_x)+" m")
    # print("\tMeters North/South: "+str(meters_y)+" m")

    # print("\tFoci distance: "+str(foci_distance)+" m")
    # print("\tDistance to target: "+str(distance_to_target)+" m")

    known_alt = float(0.624383807182312)
    known_lat = float(0.0005601330693765801)
    known_lon = float(-0.0004435136425839287)

    # print("\tEstimated target location: "+str(al)+" alt, " + str(lat) + " lat, " + str(lon) + " lon")

    props = preprocessing.normalize([np.array([xC, yC, bbw, bbh])])

    time.sleep(1)

    return 1, lat, lon