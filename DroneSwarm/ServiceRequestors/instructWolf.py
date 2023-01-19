
import rospy
from std_srvs.srv import Trigger, TriggerResponse

# Main action handler function, calls the service and prints the message
def sendWolfCommandClusterInfo(serviceName):

    print("Using service", serviceName)
    rospy.wait_for_service(serviceName)
    response = rospy.ServiceProxy(serviceName, Trigger)
    resp = response()

    print(resp.message)
    return response
