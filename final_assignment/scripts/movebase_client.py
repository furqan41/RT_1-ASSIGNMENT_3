#! /usr/bin/env python

"""


This node is a client that accept some target from the user interface and then send the target as a goal 
to the movebase action server

Subscribes to:
    /movebase_action_feedback topic where the movebase action sends a feedback of 
    the current position of the robot
    
Service:
    /movebase_client a server that gets the required target position from the user interface
    /movebase_result a server that waits until the goal has been successful and then sends a 
    response of "Target Reached" to the user interface client  
    
"""

import rospy
# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg._GoalStatus import GoalStatus

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


from final_assignment.srv import MoveBaseTarget, MoveBaseTargetResponse, MoveBaseResult, MoveBaseResultResponse
from std_srvs.srv import *
from geometry_msgs.msg import Twist


from math import *
import time

# variable that takes in the current position of the robot
current_position_x = 0.0
current_position_y = 0.0
target = MoveBaseTarget()

# variable used to specify the state of the node, each state corresponds to some action
state = 0


def the_distance_to_target(target):
    """Calculate the distance between the position of the robot and the
    target position 

    Args:
        target (
        Target): This is a custom ROS message containing an x and y 
        coordinate of the target position

    Returns:
        [float]: A float containing the distance between the robot pose
        and the target pose.  
    """

    dist_x = target.cord_x - current_position_x
    dist_y = target.cord_y - current_position_y
    distance_to_target = sqrt((dist_x * dist_x) + (dist_y * dist_y))
    return distance_to_target


def check_target(target):
    """This function check to see if the robot is not already in the new 
    target that is received. 

    Args:
        target (MoveBaseTarget): This is a custom ROS message containing an x and y 
        coordinate of the target position

    Returns:
        [bool]: The function returns True if the robot is already at the new 
        target received and returns False if the robot is not at the new target received. 
    """
    if (abs(current_position_x - target.cord_x) < 0.5 and abs(current_position_y - target.cord_y) < 0.5):
        return True
    else:
        return False


def handle_target(mes):
    """This function is a callback function for the movebase client that sets 
    the state to 1 which means active (start goal)

    Args:
        mes (MoveBaseTarget): This is a custom ROS message containing an x and y 
        coordinate of the target position

    Returns:
        [MoveBaseTargetResponse]: Sends a response status message to the client
        that called the service
    """
    global target
    global state
    if(check_target(mes)):
        return MoveBaseTargetResponse(True)
    else:
        target = mes
        state = 1
        return MoveBaseTargetResponse(False)


def handle_result(mes):
    """This is a callback function for a service for waiting for the result from 
    the movebase action server.    

    Args:
        mes (string): Containing a string message from the client that called this service

    Returns:
        [MoveBaseResultResponse]: Send a response of status message "Target Reached" 
        to the client when state is set to SUCCEEDED by the movebase action. 
    """
    time.sleep(1)
    while(state != 0):
        time.sleep(2)

    return MoveBaseResultResponse('Yes')


def feedback_callback(pose_message):
    """
    The pose callback function takes the position and posture of
    the robot from the argument "pose_message" and set it to
    two global variables containing the x and y coordinates pose.

    Args:
        pose_message (Odom): an object containing all the values
        of the current position and posture of the robot
    """

    # "global" makes the variable accessible everywhere in the code
    global current_position_x
    global current_position_y

    current_position = pose_message.base_position.pose.position

    current_position_x = current_position.x
    current_position_y = current_position.y


def movebase_client(client, target):
    """This function is responsible for calling the movebase action server
    to send the goal target and set some necessary parameter for the movebase
    action. 

    Args:
        client (Object): Object for linking to the Initialize movebase action server
        target (MoveBaseTarget): This is a custom ROS message containing an x and y 
        coordinate of the target position
    """

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"

    # Move to the target cordinate
    goal.target_pose.pose.position.x = target.cord_x
    goal.target_pose.pose.position.y = target.cord_y

    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=feedback_callback)
    # Waits for the server to finish performing the action.
    # wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
    # if not wait:
    #    rospy.logerr("Action server not available!")
    #    rospy.signal_shutdown("Action server not available!")
    return  # client.get_result()


def main():
    # Initializing the node
    rospy.init_node('movebase_client')

    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    rate = rospy.Rate(20)

    # Initializing the servers
    s = rospy.Service('movebase_client', MoveBaseTarget, handle_target)
    t = rospy.Service('movebase_result', MoveBaseResult, handle_result)
    global state
    while not rospy.is_shutdown():
        if (state == 0):
            rate.sleep()
            continue

        elif (state == 1):
            movebase_client(client, target)
            state = 2

        if (state == 2):
            time.sleep(0.5)
            distance_to_target = the_distance_to_target(target)
            print(
                f'Distance to target: {distance_to_target :.4f}, x: {current_position_x :.4f}, y: {current_position_y :.4f}')
            if (client.get_state() == GoalStatus.SUCCEEDED):
                print("\nTarget Reached !!!\n")
                state = 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
