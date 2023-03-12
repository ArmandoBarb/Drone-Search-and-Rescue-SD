import math
import numpy as np
import HelperFunctions.calcHelper as helper

# RADIUS = 0.0001
def separation(currentGPS, targetGPS, radius, maxSpeed):
    distanceTarget = helper.calcDistanceBetweenGPS(currentGPS, targetGPS);

    vector = helper.calcVectorBetweenGPS(currentGPS, targetGPS);
    vectorR = [-vector[0], -vector[1]]

    # calc 
    finalVX = 0
    finalVY = 0
    distanceTarget = abs(distanceTarget)
    radius = abs(radius)

    if (distanceTarget < radius):
        # print("In RADIUS: " + str(distanceTarget))
        powerRange = (radius - distanceTarget) / radius # 0-1
        vectorMagnitude = (powerRange * maxSpeed)  # try full power
        multiplyXY = vectorMagnitude / distanceTarget

        finalVX = vectorR[0] * multiplyXY
        finalVY = vectorR[1] * multiplyXY
    
    return [finalVX, finalVY]

def cohesion(currentGPS, targetGPS, radius, maxSpeed):
    distanceTarget = helper.calcDistanceBetweenGPS(currentGPS, targetGPS);
    vector = helper.calcVectorBetweenGPS(currentGPS, targetGPS);

    finalVX = 0
    finalVY = 0
    distanceTarget = abs(distanceTarget)
    radius = abs(radius)

    if (distanceTarget > radius):
        powerRange = (distanceTarget - radius) / distanceTarget # 0-1
        vectorMagnitude = (powerRange * maxSpeed) 
        multiplyXY = vectorMagnitude / distanceTarget

        finalVX = vector[0] * multiplyXY
        finalVY = vector[1] * multiplyXY

    return [finalVX, finalVY]

def alignmentNormal(currentGPS, targetGPS):
    vector = helper.calcVectorBetweenGPS(currentGPS, targetGPS);
    vectorR = [-vector[0], -vector[1]]

    alignmentVector = helper.perpendicularVector(helper.normalizeVector(vectorR));

    return alignmentVector;

def alignmentCentripetal(currentGPS, targetGPS, radiusM, desiredVelocity):
    vector = helper.calcVectorBetweenGPS(currentGPS, targetGPS);

    magnitude = helper.calcCentripetalVelocity(desiredVelocity, radiusM)

    vectorCentripetal = helper.setVectorMagnitude(vector, magnitude)
    return vectorCentripetal;

def alignmentMagnitude(currentGPS, alignmentVector, speed, maxBonusSpeed, radius, wolfData):
    alignmentSpeed = 0;

    droneNumber = len(wolfData) + 1;

    if (droneNumber == 1):
        return (speed + maxBonusSpeed);

    sideLength = helper.calcDroneSeparationDistance(radius, droneNumber)

    # sideLength = sideLength * 1.2; # increasing side length may cause jerkiness when circling

    finalVX = 0;
    finalVY = 0;
    vectorsAdded = 0;

    for wolf in wolfData:
        distanceToWolf = helper.calcDistanceBetweenGPS(currentGPS, wolf);
        # check if wolf is to close to currentDrone
        if (distanceToWolf < sideLength and distanceToWolf != 0):
            powerRange = (sideLength - distanceToWolf) / sideLength; # 0-1

            multiplyXY = powerRange / distanceToWolf;
            
            vector = helper.calcVectorBetweenGPS(currentGPS, wolf);
            vectorR = [-vector[0], -vector[1]];

            finalVX += vectorR[0] * multiplyXY;
            finalVY += vectorR[1] * multiplyXY;
            
            vectorsAdded += 1;

    vectorSeparation = [finalVX, finalVY]

    vectorLength = helper.calcVectorMagnitude(vectorSeparation);
    if (vectorsAdded > 0):
        vectorLength = vectorLength / vectorsAdded # 0-1
    else:
        alignmentSpeed = speed
        return alignmentSpeed; # no drone needing to be seperated from

    # check direction of seperation vector
    angle = helper.calcVectorAngle(alignmentVector, vectorSeparation);
    if (angle < 90):
        alignmentSpeed = speed + (vectorLength * maxBonusSpeed) # Speed up
    else:
        alignmentSpeed = speed - (math.sqrt(vectorLength) * speed); # slow down

    return alignmentSpeed;