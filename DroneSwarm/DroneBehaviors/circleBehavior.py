import DroneBehaviors.basicBehaviors as behaviors
import HelperFunctions.calcHelper as helper
import math

# wolfData - should not contain currentDrone
def calcSpeedVector(currentDroneData, targetGPS, radius, radiusM, wolfData, averageAlignmentSpeed, bonusAlignmentSpeed, maxCohSepSpeed, maxSpeed): # currentGps, otherDronesGPS, targetGPS
    # Add trainable parameters if needed
    #

    # Extract drone data
    currentGPS = currentDroneData.gps_location;
    linearVelocity = currentDroneData.kinematics_estimated.linear_velocity;
    currentVelocityXY = [linearVelocity.x_val, linearVelocity.y_val];

    # caclualte cohesion and seperation (orbit)
    vectorSeparation =  behaviors.separation(currentGPS, targetGPS, radius, maxCohSepSpeed);
    vectorCohesion = behaviors.cohesion(currentGPS, targetGPS, radius, maxCohSepSpeed);
    vextorX = (vectorSeparation[0] + vectorCohesion[0]);
    vextorY = (vectorSeparation[1] + vectorCohesion[1]);

    vectorOrbit = [vextorX, vextorY];

    # caclulate alignment directions, speed, and centripacal velocity
    vectorAlignmentNormal = behaviors.alignmentNormal(currentGPS, targetGPS);
    alignmentSpeed = behaviors.alignmentMagnitude(currentGPS=currentGPS, alignmentVector=vectorAlignmentNormal, speed=averageAlignmentSpeed, maxBonusSpeed=bonusAlignmentSpeed, radius=radius, wolfData=wolfData);
    
    vectorCentripetalVelocity = behaviors.alignmentCentripetal(currentGPS=currentGPS, targetGPS=targetGPS, radiusM=radiusM, desiredVelocity=alignmentSpeed);
    vectorAlignment = helper.setVectorMagnitude(vectorAlignmentNormal, alignmentSpeed);

    # Combine vectors for final vector direction

    # TODO verify if vectors should be latitude then longitude
    vextorX = vectorOrbit[0]  + vectorAlignment[0] + vectorCentripetalVelocity[0] # Longitude
    vextorY = vectorOrbit[1] + vectorAlignment[1] + vectorCentripetalVelocity[1] # Latitude
    vector = [vextorX, vextorY] 

    # Enforce maxSpeed
    return helper.applyMaxSpeed(vector, maxSpeed); # return desired movment vector


def calcYaw(currentGPS, targetGPS):
    vectorGPS = helper.calcVectorBetweenGPS(currentGPS, targetGPS);
    perpVector =  helper.perpendicularVector(helper.normalizeVector(vectorGPS));
    # calulate yaw
    yaw = math.atan2(perpVector[0], perpVector[1]);
    degrees = 360 - math.degrees(yaw); # yaw to face circle center
    return degrees;

def IsInPosition(currentGPS, targetGPS, radius, wolfData, minDiffrenceInRadius, requiredSeperationPercent): 
    distanceToTarget = helper.calcDistanceBetweenGPS(currentGPS, targetGPS)
    distanceFromDesiredPos = abs(radius - distanceToTarget)

    if (distanceFromDesiredPos > minDiffrenceInRadius):
        return False

    droneNumber = len(wolfData) + 1;
    sideLength = helper.calcDroneSeparationDistance(radius, droneNumber) * requiredSeperationPercent

    for wolf in wolfData:
        distanceToWolf = helper.calcDistanceBetweenGPS(currentGPS, wolf)
        if (distanceToWolf < sideLength):
            return False

    return True