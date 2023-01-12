import rospy
import ast
from std_srvs.srv import Trigger, TriggerResponse

PROXIMITY_OVERSEER_SERVICE = "ProximityOverseerService"

def getOverseerState():
    # Get overseer data using a service
    rospy.wait_for_service(PROXIMITY_OVERSEER_SERVICE)

    # Gets service response and messsage from OverseerData
    response = rospy.ServiceProxy(PROXIMITY_OVERSEER_SERVICE, Trigger)
    resp = response()
    responseText = resp.message
    convertedResponseArray = ast.literal_eval(responseText)

    return convertedResponseArray
