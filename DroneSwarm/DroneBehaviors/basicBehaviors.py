import math

def separation(currentGPS, targetGPS):
    # range
    radius = 0.0002 # 0.00005
    AVOID_FACTOR = 7000

    currentLongitude = currentGPS.longitude
    currentLatitude = currentGPS.latitude

    targetLongitude = targetGPS.longitude
    targetLatitude = targetGPS.latitude
    # Variables used for separation for drones too close
    finalVX = 0
    finalVY = 0

    longitudeDifference = currentLongitude - targetLongitude
    latitudeDifference = currentLatitude - targetLatitude

    distanceTarget = math.sqrt( (longitudeDifference ** 2) + (latitudeDifference ** 2) )

    if (distanceTarget < radius):
        print("In radius: " + distanceTarget)
        finalVX += targetLongitude * AVOID_FACTOR
        finalVY += targetLatitude * AVOID_FACTOR
    
    return [finalVX, finalVY]

def cohesion(currentGPS, targetGPS):
    CENTERING_FACTOR = 20000

    currentLongitude = currentGPS.longitude
    currentLatitude = currentGPS.latitude

    targetLongitude = targetGPS.longitude
    targetLatitude = targetGPS.latitude

    finalVX = (targetLongitude - currentLongitude)*CENTERING_FACTOR
    finalVY = (targetLatitude - currentLatitude)*CENTERING_FACTOR

    return [finalVX, finalVY]

def alignment(currentGPS, targetGPS):
    # Gets average swarm velocity in x and y
    
        #averageVX = averageVXCalculator()
        #averageVY = averageVYCalculator()

    # sumVX = sumVXCalculator()
    # sumVX = sumVXCalculator()

    # Added 1 to difference to push drones towards x direction
    
        #finalVX = (averageVX)*MATCHING_FACTOR
        #finalVY = (averageVY)*MATCHING_FACTOR

    return [finalVX, finalVY]
