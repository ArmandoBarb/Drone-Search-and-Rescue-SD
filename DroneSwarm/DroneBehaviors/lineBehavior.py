import airsim
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.msg import droneData
from ServiceRequestors.wolfGetWolfData import getWolfState

AVOID_FACTOR = 1
DIRECTION_FACTOR = 10000
REPULSION_RADIUS = 0.00005
# ADD IN GLOBAL VARIABLE FOR WOLF DRONES IN OVERSEER GROUP

def repulsion(client, curDroneIndex):
    wolfInfoArray = getWolfState()        # Get droneWolfState state array from service

    # Variables used for separation for drones too close
    close_dx = 0
    close_dy = 0
    finalVX = 0
    finalVY = 0

    for nearbyDroneIndex in range(6):  # NEEDS ADJUSTMENT FOR DRONES IN GROUP
        # Get difference in location from drones
        xDifference = wolfInfoArray[curDroneIndex].longitude - wolfInfoArray[nearbyDroneIndex].longitude
        yDifference = wolfInfoArray[curDroneIndex].latitude - wolfInfoArray[nearbyDroneIndex].latitude

        # Check if any nearby drones are too close using difference values
        if ((nearbyDroneIndex != curDroneIndex) and (abs(xDifference) < REPULSION_RADIUS) and (abs(yDifference) < REPULSION_RADIUS)):
            close_dx += xDifference
            close_dy += yDifference

    # Account for updated repulsion equation, if closer have more weight, if father have less weight
    if ((close_dx != 0) and (close_dy != 0)):
        close_dx = ((abs(close_dx)/REPULSION_RADIUS) - 1) * ((abs(close_dx))/close_dx)
        close_dy = ((abs(close_dy)/REPULSION_RADIUS) - 1) * ((abs(close_dy))/close_dy)

    finalVX += close_dx*AVOID_FACTOR
    finalVY += close_dy*AVOID_FACTOR
    return [finalVX, finalVY]

def waypointDirection(client, curDroneIndex, waypoint):
    dronesAtWaypoint = 0
    wolfInfoArray = getWolfState()

    # Check if current drone is at waypoint
    xDifference = float(waypoint[0]) - wolfInfoArray[curDroneIndex].longitude
    yDifference = float(waypoint[1]) - wolfInfoArray[curDroneIndex].latitude

    # print("Waypoint longitude", waypoint[0], "Wolf longitude", wolfInfoArray[curDroneIndex].longitude, "Waypoint latitude ", waypoint[1],  wolfInfoArray[curDroneIndex].latitude, "Wolf latitude")

    # If at waypoint, don't move
    if ((abs(xDifference) < 0.0001) and (abs(yDifference) < 0.0001)):
        # print("Drone:", curDroneIndex, "At waypoint")
        finalVelocity = [0, 0]
    # Else move to waypoint
    else:
        xDifference = (float(waypoint[0]) - wolfInfoArray[curDroneIndex].longitude)*DIRECTION_FACTOR
        yDifference = (float(waypoint[1]) - wolfInfoArray[curDroneIndex].latitude)*DIRECTION_FACTOR
        finalVelocity = [xDifference, yDifference]

    return finalVelocity


def lineBehavior(client, curDroneIndex, DM_Wolfs_Cluster, waypoint_coords):
    wolfInfoArray = getWolfState()
    # If there currently is not droneData, return 0
    if (wolfInfoArray[0].droneName == ""):
        print("No drone data")
        return [0, 0]
    velocityR = repulsion(client, curDroneIndex)
    velocityD = waypointDirection(client, curDroneIndex, waypoint_coords)
    finalVelocityX = velocityD[0] + velocityR[0]
    finalVelocityY = velocityD[1] + velocityR[1]
    vector = [finalVelocityX, finalVelocityY]
    # print("Drone :", curDroneIndex, "Alignment: ", velocityA, "Cohesion: ", velocityC, "Separation: ", velocityS, "Final V: ", vector)
    return vector
