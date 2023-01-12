# Action Handler Nodes
# Each overseer will have its own "ActionHandlerX" node
# The wolf drones will reciever a goal and return feedback to the action handler
# Replies to the services of the overseer that takes over it

# Goal example: Provides a goal and recieves feedback to (WolfXGoalHandler) action
# Replies to the services of the overseer that takes over it on (OverseerXActionHandlerAPI)
# Subgoals: Search this point of interest
# If a drone were to be in progress of another goal, let the overseer know

# ROS Action
# Used as a long term api task
# Can send actions through a call
# Can check that call to gain status of request
# The node will do the action and reply the progress on it
# Action handler constantly checks the drones states
# Helps with management

import rospy
from std_srvs.srv import Trigger, TriggerResponse

# Main action handler function, calls the service and prints the message
def sendWolfCommandClusterInfo(serviceName):

    print("Using service", serviceName)
    rospy.wait_for_service(serviceName)
    #// test commit (DONT DO DAT!!!)
    response = rospy.ServiceProxy(serviceName, Trigger)
    resp = response()

    print(resp.message)
    return response
