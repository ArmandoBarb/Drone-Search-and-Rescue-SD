import rospy
import ast
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.srv import getDroneData
from std_srvs.srv import Trigger, TriggerResponse

PROXIMITY_WOLF_SERVICE = "PromixityWolfService"

def getWolfState():
    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)

    # Gets service response and droneDataArray from WolfData
    response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, getDroneData)
    resp = response()
    responseText = resp.droneDataArray

    return responseText
