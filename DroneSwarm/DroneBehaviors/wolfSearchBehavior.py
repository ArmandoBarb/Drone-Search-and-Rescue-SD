import DroneBehaviors.basicBehaviors as behaviors

def wolfSearchBehavior(currentGPS, targetGPS): # currentGps, otherDronesGPS, targetGPS
    # add cohesion
    # seperation
    # alignmetn
    print("currentGPS: " + str(currentGPS))
    print("targetGPS: " + str(targetGPS))
    vectorSeparation = behaviors.separation(currentGPS, targetGPS)
    vectorCohesion = behaviors.cohesion(currentGPS, targetGPS)
    
    print("vectorSeparation: " + str(vectorSeparation) + "vectorCohesion: " + str(vectorCohesion))
    # Longitude, Latitude
    vextorX = vectorSeparation[0] + vectorCohesion[0]
    vextorY = vectorSeparation[1] + vectorCohesion[1]
    vector = [vextorX, vextorY]
    return vector; # return desired movment vector

# calculate behaviors: -30
# client.moveByVelocityZAsync(vector[0], vector[1], -30, duration = 5, vehicle_name=droneName)
# vector x, y 
# z = height
