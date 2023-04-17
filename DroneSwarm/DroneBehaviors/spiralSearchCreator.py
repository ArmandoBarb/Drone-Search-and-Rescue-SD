import Constants.configDrones as configDrones

SPIRAL_LOCATION_1 = configDrones.SPIRAL_LOCATION_1

def spiralSearchCoordinateMaker(groupName, waypointDistance, spawnLocation, startLocation, amountOfWaypoints):
    waypointsList = [None]*amountOfWaypoints        # List creation for waypoints
    currentWaypoint = startLocation                 # Sets current waypoint as our start location
    distance = waypointDistance                     # Distance between waypoints

    with open(groupName, 'w') as f:
        direction = 0 # Direction show via int, 1 = upward, 2 = right, 3 = downward, 4 = left
        currentWaypointText = str(currentWaypoint[0]) + " " + str(currentWaypoint[1]) + "\n"
        spawnPointText = str(spawnLocation[0]) + " " + str(spawnLocation[1]) + "\n"
        f.write(spawnPointText)
        f.write(currentWaypointText)

        # Used to change starting direction
        direction = 4

        for generalWaypoints in range(int(amountOfWaypoints)):
            waypointCount = generalWaypoints + 1 # Gets number of needed waypoints for direction

            # As for spiral searches, they will go the same distance twice, turning each time
            for num in range(2):
                direction += 1 # Update waypoint
                if (direction > 4): # Reset direction
                    direction = 1

                for waypoint in range(waypointCount):
                    if (direction == 1): # Handle upward movement
                        longitude = float(currentWaypoint[0])
                        latitude = float(currentWaypoint[1]) + distance
                        currentWaypoint = [longitude, latitude]

                    elif (direction == 2): # Handle rightward movement
                        longitude = float(currentWaypoint[0]) + distance
                        latitude = float(currentWaypoint[1])
                        currentWaypoint = [longitude, latitude]

                    elif (direction == 3): # Handle downward movement
                        longitude = float(currentWaypoint[0])
                        latitude = float(currentWaypoint[1]) - distance
                        currentWaypoint = [longitude, latitude]

                    elif (direction == 4): # Handle leftward movement
                        longitude = float(currentWaypoint[0]) - distance
                        latitude = float(currentWaypoint[1])
                        currentWaypoint = [longitude, latitude]

                # Writes output to file
                outputString = str(currentWaypoint[0]) + " " + str(currentWaypoint[1])
                f.write(outputString)
                f.write('\n')

    f.close()

def createWaypoints():
    # Creates waypoints for group 0 to move to
    waypointDistance = 0.0003
    spawnLocation = [0.0001, 0.0001]
    centerStartLocation = SPIRAL_LOCATION_1
    amountOfWaypoints = 5
    spiral0Filename = 'Constants/Group0Spiral.txt'
    spiralSearchCoordinateMaker(spiral0Filename, waypointDistance, spawnLocation, centerStartLocation, amountOfWaypoints)

    # Creates waypoints for group 1 to move to
    waypointDistance = 0.0004                                   # Distance between waypoints
    spawnLocation = [-0.0001, 0.0001]
    centerStartLocation = [-0.0011228941075, 0.0011228941075]                     # Starting center of the spiral
    amountOfWaypoints =  5                                       # Amount of edges
    spiral1Filename = 'Constants/Group1Spiral.txt'
    spiralSearchCoordinateMaker(spiral1Filename, waypointDistance, spawnLocation, centerStartLocation, amountOfWaypoints)
