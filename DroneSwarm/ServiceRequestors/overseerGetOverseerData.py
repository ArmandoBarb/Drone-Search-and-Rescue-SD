import rospy
import ast
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.srv import getDroneData
from std_srvs.srv import Trigger, TriggerResponse
# import constants
import Constants.ros as ros

PROXIMITY_OVERSEER_SERVICE = ros.PROXIMITY_OVERSEER_SERVICE

def getOverseerState():
    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_OVERSEER_SERVICE)

    # Gets service response and droneDataArray from WolfData
    response = rospy.ServiceProxy(PROXIMITY_OVERSEER_SERVICE, getDroneData)
    resp = response()
    responseText = resp.droneDataArray

    return responseText
