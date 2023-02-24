import rospy
import ast
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.srv import getDroneData
from std_srvs.srv import Trigger, TriggerResponse
# import constants
import Constants.ros as ros

PROXIMITY_WOLF_SERVICE = ros.PROXIMITY_WOLF_SERVICE

def getWolfState():
    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)

    # Gets service response and droneDataArray from WolfData
    response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, getDroneData)
    resp = response()
    responseText = resp.droneDataArray

    return responseText

# Gets wolf data of cluster, excluded current drone
def getWolfDataOfCluster(wolfNameToExclude, clusterName):
    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)

    # Gets service response and droneDataArray from WolfData
    response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, getDroneData)
    resp = response()

    responseText = []
    for x in resp.droneDataArray:
        if ((x.droneName != wolfNameToExclude) and (x.cluster == clusterName)):
            responseText.append(x)

    return responseText

def getWolfDataExC(wolfNameToExclude):
    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)

    # Gets service response and droneDataArray from WolfData
    response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, getDroneData)
    resp = response()

    responseText = []
    for x in resp.droneDataArray:
        if (x.droneName != wolfNameToExclude):
            responseText.append(x)

    return responseText
