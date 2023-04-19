import rospy
import ast
from airsim_ros_pkgs.msg import GPS
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

# Gets wolf data of cluster
def getWolfDataOfClusterWCurWolf(clusterName):
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

# Gets wolf data of taskGroup, excluded current drone
def getWolfDataOfTaskGroupExSelf(wolfNameToExclude, taskGroup):
    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)

    # Gets service response and droneDataArray from WolfData
    response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, getDroneData)
    resp = response()

    responseText = []
    for x in resp.droneDataArray:
        if ((x.droneName != wolfNameToExclude) and (x.taskGroup == taskGroup)):
            responseText.append(x)

    return responseText

def getWolfDataOfTaskGroup(taskGroup):
    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)

    # Gets service response and droneDataArray from WolfData
    response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, getDroneData)
    resp = response()

    responseText = []
    for x in resp.droneDataArray:
        if ((x.taskGroup == taskGroup)):
            responseText.append(x)

    return responseText

def isTaskGroupSameIteration(taskGroup):
    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)

    # Gets service response and droneDataArray from WolfData
    response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, getDroneData)
    resp = response()

    iterationNumber = None
    for x in resp.droneDataArray:
        if ((x.taskGroup == taskGroup)):
            if(iterationNumber == None):
                iterationNumber = x.iterationNumber
            if (iterationNumber != x.iterationNumber):
                return False
    return True

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
        if (x.cluster == clusterName):
            droneCount += 1;
            clusterCenterGPS.longitude += x.longitude
            clusterCenterGPS.latitude += x.latitude

    if (droneCount == 0): return True, None
    clusterCenterGPS.longitude = clusterCenterGPS.longitude / droneCount
    clusterCenterGPS.latitude = clusterCenterGPS.latitude / droneCount

    # print(clusterName, "Drone Count:", droneCount, clusterCenterGPS)

    return False, clusterCenterGPS