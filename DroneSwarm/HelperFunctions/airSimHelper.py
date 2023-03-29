from math import sqrt
import airsim

def getDroneSpeed(client, droneName):
    velocity = client.getGpsData(vehicle_name = droneName)

    velocityX = velocity.gnss.velocity.x_val
    velocityY = velocity.gnss.velocity.y_val

    speed = sqrt(velocityX**2 + velocityY**2)

    return speed