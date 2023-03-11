def getOptimalWolf(waypoint, wolfList, clusterName):
    shortestDistance = 10000000
    optimalDrone = ""

    for drone in wolfList:
        # Get drones distance to waypoint
        curDroneDistance = ( (float(waypoint[0]) - drone.longitude)**2 + (float(waypoint[1]) - drone.latitude)**2 )**0.5

        # If drone is close and available, save the drone name
        if ((curDroneDistance < shortestDistance) and (drone.taskGroup == "") and (drone.cluster == clusterName)):
            optimalDrone = drone.droneName
            shortestDistance = curDroneDistance

    return optimalDrone