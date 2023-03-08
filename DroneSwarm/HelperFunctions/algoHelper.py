def getOptimalWolf(waypoint, wolfList):
    shortestDistance = 10000000
    optimalDrone = ""

    for drone in wolfList:
        # Get drones distance to waypoint
        curDroneDistance = sqrt( (float(waypoint[0]) - drone.longitude)**2 + (float(waypoint[1]) - drone.latitude)**2 )

        # If drone is close and available, save the drone name
        if ((curDroneDistance < shortestDistance) and (drone.taskGroup == "") and (drone.cluster == clusterName)):
            optimalDrone = drone.droneName
            shortestDistance = curDroneDistance

    return optimalDrone