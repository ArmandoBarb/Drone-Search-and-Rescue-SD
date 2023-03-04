import rospy
import ast
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
