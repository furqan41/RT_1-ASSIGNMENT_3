#! /usr/bin/env python

"""


This is the user interface that prints a prompt message of a number of action that can be 
performed by the user and then set the state of the node base on the response gotten from the user

Publishes to: 
    /cmd_vel used to stop the motion of the robot on request of the user 
    
Service:
    /
   
   
    /movebase_client request the movebase client node to move the robot to 
    target location with the movebase action server 
    /movebase_result request the movebase result service to wait until the robot
    has reached the goal target. 
    /bug_switch request the bug0 algorithm to control the robot to reach a target
    /wall_follower_switch request the wall follower node to make the robot 
    follow the walls in the simulation environment
    
"""

import rospy
import roslaunch

from final_assignment.srv import MoveBaseTarget, MoveBaseResult
from std_srvs.srv import *
from geometry_msgs.msg import Twist

from math import *
import time
import random

# List containing all the possible target location in the simulation
TARGET_POSE = [(-4, -3), (-4, 2), (-4, 7), (5, -7), (5, -3), (5, 1)]

target2 = MoveBaseTarget()
algo = 0
path_planner = ['Move Base: Dijkstra', 'Bug0']
task_open = False

def check_location(x, y):
    """This function checks to see if the location selected by the user
    is one of the locations contained in posible target position list

    Args:
        x (int): The x coordinate of the selected position
        y (int): The y coordinate of the selected position

    Returns:
        [bool]: The function returns True if the position selected 
        is in the posible target position list, and False if it is
        not in the list. 
    """
    if (x, y) in TARGET_POSE:
        return True
    else:
        return False

def start_task():
    rospy.loginfo("starting...")
    
    package ="teleop_twist_keyboard"
    
    executable ="teleop_twist_keyboard.py"
    node = roslaunch.core.Node(package, executable)
    
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    task = launch.launch(node)
    print(task.is_alive())



def call_movebase(target):
    """This function takes in a target position and sends it as a 
    request to a service called movebase client, this service communicates
    with the movebase action server to perform path planning task for moving
    the robot from the current position to the target position. 

    Args:
        target (MoveBaseTarget): This is a custom ROS message containing an x and y 
        coordinate of the target position

    Returns:
        [string]: A string containing a response message sent from the server. 
    """
    rospy.wait_for_service('movebase_client')
    try:
        movebase_client = rospy.ServiceProxy('movebase_client', MoveBaseTarget)
        res = movebase_client(target.cord_x, target.cord_y)
        return res.status
    except rospy.ServiceException as e:
        print(f'Service call failed: {e}')


def call_bug_algo(target):
    """This function calls the node for the bug0 algorithm, it is called 
    when the user switches to bug0 algorithm. It sends a target position to the
    bug0 node by the way of ros parameter server and receives a response
    of status message

    Args:
        target (MoveBaseTarget): This is a custom ROS message containing an x and y 
        coordinate of the target position

    Returns:
        [string]: A string containing a response message sent from the server. 
    """

    # sends the x and y coordinate to a parameter server
    rospy.set_param("des_pos_x", target.cord_x)
    rospy.set_param("des_pos_y", target.cord_y)
    print("Thanks! Let's reach the next position")
    rospy.wait_for_service('bug_switch')
    try:
        bug0 = rospy.ServiceProxy('bug_switch', MoveBaseResult)
        res = bug0('Are you there yet!')
        return res.status
    except rospy.ServiceException as e:
        print(f'Service call failed: {e}')


def wait_for_result():
    """This function sends a request to a movebase result server to wait until
    the target had been reached before send a response of target reached. 

    Returns:
        [string]: A string containing a response message "Target Reached". 
    """
    rospy.wait_for_service('movebase_result')
    try:
        movebase_result = rospy.ServiceProxy('movebase_result', MoveBaseResult)
        res = movebase_result('Are you there yet!')
        return res.status
    except rospy.ServiceException as e:
        print(f'Service call failed: {e}')




def main():
    """The main function for initializing the node an calling each of the 
    node functions based on the input from the user. The main function makes 
    use of state value to select the appropriate function for the required task 
    """
    # Initializing the node
    rospy.init_node('user_interface')

    # Creating a publisher object
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    global algo
    state = 0
    pick = 0
    global task_open
    global TARGET_POSE

    while not rospy.is_shutdown():
        # Prompt message to inform the user of the actions that can be performed
        if task_open == False:
            prompt_mes = f"""
        
        
        
Please select the following mode for controlling the robot:

Possible Position inside the MAP boundry = {TARGET_POSE}


1. Switch to Bug0 algorithm.
2. Enter the  one of the possible positions with in the MAP boundry
3. Control the Robot with the key_board Manually .

        """
        elif task_open == True:
            prompt_mes = """Controlling manually now using teleop    i
                                       j k l """
            
            
        if (state == 0):
            print(prompt_mes)
            try:
                pick = int(input("Enter the number here: "))
                state = 1
            except ValueError:
                print('\nPLEASE ENTER A VALID MODE !!!\n')
                state = 0

        if (pick == 1 and state == 1):
            target = call_bug_algo(random.choice(TARGET_POSE))
            if (algo == 0):
                resp = call_movebase(target)
                while (resp):
                    target = call_bug_algo()
                    resp = call_movebase(target)
                wait_for_result()
                state = 0
            else:
                resp = call_bug_algo(target2)
                print(resp)
                algo = 0
                state = 0

        elif (pick == 2 and state == 1):
            print('Please enter a x and y cordinate from the possible position list')
            x = int(input('x: '))
            y = int(input('y: '))
            if (check_location(x, y)):
                target2.cord_x = x
                target2.cord_y = y
                if (algo == 0):
                    resp = call_movebase(target2)
                    if(resp):
                        print('The Robot is already at this location')
                    else:
                        wait_for_result()
                        state = 0
                else:
                    resp = call_bug_algo(target2)
                    print(resp)
                    algo = 0
                    state = 0
            else:
                print('Please enter one of the possible positions')
                
        elif (pick == 3 and state == 1 and task_open==False):
        	print("Opening Teleop")
        	start_task()
        	task_open = True
        	

        
      
        elif (pick not in range(1, 3)):
            state = 0
            


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

