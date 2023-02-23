
import rospy
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.msg import requestLineBehavior
from airsim_ros_pkgs.msg import requestWolfSearchBehavior
from airsim_ros_pkgs.msg import requestConsensusDecisionBehavior
from airsim_ros_pkgs.srv import getDroneData
from airsim_ros_pkgs.srv import sendCommand

# Send service to start live behavior with given waypoints
def requestLineBehavior(serviceName, GROUP_0_SEARCH, GROUP_1_SEARCH):
    # Create messages needed with parameters
    messageType = "RequestLineBehavior"
    linebehaviorMsg = requestLineBehavior()
    linebehaviorMsg.group0Waypoints = GROUP_0_SEARCH
    linebehaviorMsg.group1Waypoints = GROUP_1_SEARCH
    # Add empty data for unused message
    wolfSearchBehaviorMsg = requestWolfSearchBehavior()
    consensusDecisionBehaviorMsg = requestConsensusDecisionBehavior()

    # Sends service
    print("requestLineBehavior from wolf: ", serviceName)
    rospy.wait_for_service(serviceName)
    response = rospy.ServiceProxy(serviceName, sendCommand)
    resp = response(messageType, linebehaviorMsg, wolfSearchBehaviorMsg, consensusDecisionBehaviorMsg)

    # Prints bool on whether behavior was passed or not
    print("Message status: ", resp, " wolf: ", serviceName)
    return response

def requestWolfSearchBehavior(serviceName, circleCenterGPS, circleRadiusGPS, circleRadiusMeters, spreadTimeS, searchTimeS):
    # Create messages needed with parameters
    messageType = "RequestWolfSearch"
    wolfSearchBehaviorMsg = requestWolfSearchBehavior()
    wolfSearchBehaviorMsg.circleCenterGPS = circleCenterGPS
    # circleCenterGPS
    # wolfSearchBehaviorMsg.circleCenterGPS.latitude = circleCenterGPS.latitude
    # wolfSearchBehaviorMsg.circleCenterGPS.longitude = circleCenterGPS.longitude
    # wolfSearchBehaviorMsg.circleCenterGPS.altitude = circleCenterGPS.altitude
    # wolfSearchBehaviorMsg.circleCenterGPS.yaw = circleCenterGPS.yaw

    wolfSearchBehaviorMsg.circleRadiusGPS = circleRadiusGPS
    wolfSearchBehaviorMsg.circleRadiusMeters = circleRadiusMeters
    wolfSearchBehaviorMsg.spreadTimeS = spreadTimeS
    wolfSearchBehaviorMsg.searchTimeS = searchTimeS

    # Add empty data for unused message
    linebehaviorMsg = requestLineBehavior()
    consensusDecisionBehaviorMsg = requestConsensusDecisionBehavior()

    # Sends service
    print("requestWolfSearchBehavior from wolf: ", serviceName)
    rospy.wait_for_service(serviceName)
    response = rospy.ServiceProxy(serviceName, sendCommand)
    resp = response(messageType, linebehaviorMsg, wolfSearchBehaviorMsg, consensusDecisionBehaviorMsg)

    # Prints bool on whether behavior was passed or not
    print("Message status: ", resp, " wolf: ", serviceName)
    return response

def requestConsensusDecisionBehavior(serviceName, circleCenterGPS, circleRadiusGPS, circleRadiusMeters, searchTimeS):
    # Create messages needed with parameters
    messageType = "RequestConsensusDecision"
    consensusDecisionBehaviorMsg = requestConsensusDecisionBehavior()

    consensusDecisionBehaviorMsg.circleCenterGPS = circleCenterGPS
    # circleCenterGPS
    # consensusDecisionBehaviorMsg.circleCenterGPS.latitude = circleCenterGPS.latitude
    # consensusDecisionBehaviorMsg.circleCenterGPS.longitude = circleCenterGPS.longitude
    # consensusDecisionBehaviorMsg.circleCenterGPS.altitude = circleCenterGPS.altitude
    # consensusDecisionBehaviorMsg.circleCenterGPS.yaw = circleCenterGPS.yaw

    consensusDecisionBehaviorMsg.circleRadiusGPS = circleRadiusGPS
    consensusDecisionBehaviorMsg.circleRadiusMeters = circleRadiusMeters
    consensusDecisionBehaviorMsg.searchTimeS = searchTimeS

    # Add empty data for unused message
    linebehaviorMsg = requestLineBehavior()
    wolfSearchBehaviorMsg = requestWolfSearchBehavior()

    # Sends service
    print("requestConsensusDecisionBehavior from wolf: ", serviceName)
    rospy.wait_for_service(serviceName)
    response = rospy.ServiceProxy(serviceName, sendCommand)
    resp = response(messageType, linebehaviorMsg, wolfSearchBehaviorMsg, consensusDecisionBehaviorMsg)

    # Prints bool on whether behavior was passed or not
    print("Message status: ", resp, " wolf: ", serviceName)