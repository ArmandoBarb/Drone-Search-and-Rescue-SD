import DroneBehaviors.lineBehaviorWolf as lineBehaviorWolf

# Function get drones subwaypoint based on index
def getNewWaypointWolf(droneName, waypointIndex, waypointCoords, Cluster):
    # Grabs current waypoint from coordinate list
    currentWaypoint = waypointCoords[waypointIndex]

    # Calculates subwaypoint if past first spawn waypoint
    if (waypointIndex >= 1):
        previousWaypoint = waypointCoords[waypointIndex-1]
        radius = 0.0001
        currentWaypoint = lineBehaviorWolf.subWaypointCalculator(currentWaypoint, previousWaypoint, radius, droneName, Cluster)

    return currentWaypoint

# TODO: MOVE TO HELPER

# Grabs subwaypoint based on waypoints and droneName
def getLastWaypointWolf(droneName, waypointIndex, waypointCoords, Cluster):
    
    # print("DroneName: ", droneName, "Current waypoint index", WAYPOINT_INDEX)
    if (waypointIndex < 2):
        currentWaypoint = waypointCoords[0]
    else:
        currentWaypoint = waypointCoords[waypointIndex - 1]

        previousWaypoint = waypointCoords[waypointIndex - 2]
        radius = 0.0001
        currentWaypoint = lineBehaviorWolf.subWaypointCalculator(currentWaypoint, previousWaypoint, radius, droneName, Cluster)

    return currentWaypoint