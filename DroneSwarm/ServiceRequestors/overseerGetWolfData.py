import rospy
import ast
import math
from math import sqrt
from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.msg import droneData
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

def getOptimalWolf(waypoint, clusterName):
    shortestDistance = 10000000
    optimalDrone = ""

    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)

    # Gets service response and droneDataArray from WolfData
    response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, getDroneData)
    resp = response()

    responseText = []
    for drone in resp.droneDataArray:
        # Get drones distance to waypoint
        curDroneDistance = sqrt( (float(waypoint[0]) - drone.longitude)**2 + (float(waypoint[1]) - drone.latitude)**2 )

        # If drone is close and available, save the drone name
        if ((curDroneDistance < shortestDistance) and (drone.taskGroup == "") and (drone.cluster == clusterName)):
            optimalDrone = drone.droneName
            shortestDistance = curDroneDistance

    return optimalDrone
