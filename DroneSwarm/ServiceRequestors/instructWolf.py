
import rospy
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.msg import lineBehavior
from airsim_ros_pkgs.msg import wolfSearchBehavior
from airsim_ros_pkgs.msg import consensusDecisionBehavior
from airsim_ros_pkgs.srv import getDroneData
from airsim_ros_pkgs.srv import sendCommand

# Send service to start live behavior with given waypoints
def sendStartLineBehavior(serviceName, GROUP_0_SEARCH, GROUP_1_SEARCH):
    # Create messages needed with parameters
    messageType = "RequestLineBehavior"
    linebehaviorMsg = lineBehavior()
    linebehaviorMsg.group0Waypoints = GROUP_0_SEARCH
    linebehaviorMsg.group1Waypoints = GROUP_1_SEARCH
    wolfSearchBehaviorMsg = wolfSearchBehavior()
    consensusDecisionBehaviorMsg = consensusDecisionBehavior()

    # Sends service
    print("Using service", serviceName)
    rospy.wait_for_service(serviceName)
    response = rospy.ServiceProxy(serviceName, sendCommand)
    resp = response(messageType, linebehaviorMsg, wolfSearchBehaviorMsg, consensusDecisionBehaviorMsg)

    # Prints bool on whether behavior was passed or not
    print("Message status: ", resp)
    return response