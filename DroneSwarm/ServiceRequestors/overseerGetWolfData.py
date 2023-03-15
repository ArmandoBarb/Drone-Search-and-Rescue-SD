import rospy
import ast
import math
from math import sqrt
from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.msg import GPS
from airsim_ros_pkgs.srv import getDroneData
# import constants
import Constants.ros as ros

PROXIMITY_WOLF_SERVICE = ros.PROXIMITY_WOLF_SERVICE

def getOverseerGetWolfState():
    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)

    # Gets service response and droneDataArray from WolfData
    response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, getDroneData)
    resp = response()
    responseText = resp.droneDataArray

    return responseText

# Gets wolf data of cluster, excluded current drone
def getWolfDataOfCluster(clusterName):
    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)
    
     # Gets service response and droneDataArray from WolfData
    response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, getDroneData)
    resp = response()
    
    responseText = []
    for x in resp.droneDataArray:
        if (x.cluster == clusterName):
            responseText.append(x)

    return responseText

def getWolfClusterCenterGPS(clusterName):
    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)
    
     # Gets service response and droneDataArray from WolfData
    response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, getDroneData)
    resp = response()
    
    droneCount = 0
    clusterCenterGPS = GPS()
    clusterCenterGPS.longitude = 0
    clusterCenterGPS.latitude = 0

    for x in resp.droneDataArray:
        # print("clusterName: ", clusterName, " x: ", x.cluster)
        if (x.cluster == clusterName):
            droneCount += 1;
            clusterCenterGPS.longitude += x.longitude
            clusterCenterGPS.latitude += x.latitude

    if (droneCount == 0): return True, None
    clusterCenterGPS.longitude = clusterCenterGPS.longitude / droneCount
    clusterCenterGPS.latitude = clusterCenterGPS.latitude / droneCount

    # print(clusterName, "Drone Count:", droneCount, clusterCenterGPS)

    return False, clusterCenterGPS

# def getOptimalWolf(waypoint, clusterName):
#     shortestDistance = 10000000
#     optimalDrone = ""

#     # Get wolf data using a service
#     rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)

#     # Gets service response and droneDataArray from WolfData
#     response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, getDroneData)
#     resp = response()

#     for drone in resp.droneDataArray:
#         # Get drones distance to waypoint
#         curDroneDistance = sqrt( (float(waypoint[0]) - drone.longitude)**2 + (float(waypoint[1]) - drone.latitude)**2 )

#         # If drone is close and available, save the drone name
#         if ((curDroneDistance < shortestDistance) and (drone.taskGroup == "") and (drone.cluster == clusterName)):
#             optimalDrone = drone.droneName
#             shortestDistance = curDroneDistance

#     return optimalDrone
