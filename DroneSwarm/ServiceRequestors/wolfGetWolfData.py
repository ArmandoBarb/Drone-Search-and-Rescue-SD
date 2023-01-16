import rospy
import ast
from std_srvs.srv import Trigger, TriggerResponse
# import constants
import Constants.ros as ros

PROXIMITY_WOLF_SERVICE = ros.PROXIMITY_WOLF_SERVICE

def getWolfState():
    # Get wolf data using a service
    rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)

    # Gets service response and messsage from WolfData
    response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, Trigger)
    resp = response()
    responseText = resp.message
    convertedResponseArray = ast.literal_eval(responseText)

    return convertedResponseArray
