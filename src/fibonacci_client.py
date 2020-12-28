#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# uint8 PENDING         = 0   # The goal has yet to be processed by the action server
# uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
# uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
#                             #   and has since completed its execution (Terminal State)
# uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
# uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
#                             #    to some failure (Terminal State)
# uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
#                             #    because the goal was unattainable or invalid (Terminal State)
# uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
#                             #    and has not yet completed execution
# uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
#                             #    but the action server has not yet confirmed that the goal is canceled
# uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
#                             #    and was successfully cancelled (Terminal State)
# uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
#                             #    sent over the wire by an action server

PENDING = 0
ACTIVE = 1
PREEMPTED = 2
SUCCEEDED = 3
ABORTED = 4

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg

waypoints = [
	[(-3.6424, -3.4765, 0.0000), (0.0000, 0.0000, 0.2121, 0.9772)],
	[(1.1133, -2.5329, 0.0000), (0.0000, 0.0000, 0.5976, 0.8018)],
	[(3.2502, 2.3253, 0.0000),(0.0000, 0.0000, 0.9831, 0.1829)],
	[(-1.1888, 3.8447, 0.0000), (0.0000, 0.0000, -0.5554, 0.8316)],
	[(3.3542, 7.1520, 0.0000),(0.0000, 0.0000, 0.9933, 0.1553)],
	[(-2.4514, 7.4468, 0.0000, 0.0000),(0.0000, 0.0000, 0.1949, 0.9808)]
]

def goal_pose(pose):
	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = 'map'
	goal_pose.target_pose.pose.position.x = pose[0][0]
	goal_pose.target_pose.pose.position.y = pose[0][1]
	goal_pose.target_pose.pose.position.z = pose[0][2]
	goal_pose.target_pose.pose.orientation.x = pose[1][0]
	goal_pose.target_pose.pose.orientation.y = pose[1][1]
	goal_pose.target_pose.pose.orientation.z = pose[1][2]
	goal_pose.target_pose.pose.orientation.w = pose[1][3]
	
	return goal_pose

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)
    client.wait_for_server()
    client_2 = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client_2.wait_for_server()

    # Waits until the action server has started up and started
    # listening for goals.
    

    # Creates a goal to send to the action server.
    goal = actionlib_tutorials.msg.FibonacciGoal(order=20)
    goal_2 = goal_pose(waypoints[0])
    
    client.send_goal(goal)
    client_2.send_goal(goal_2)
    # Sends the goal to the action server.


    # Waits for the server to finish performing the action.
    # client.wait_for_result()
    status = client.get_state()
    status_2 = client_2.get_state()

    while status < SUCCEEDED:

        rospy.loginfo('Yendo a punto')
        status_2 = client_2.get_state()
        status = client.get_state()
        print("Status:",status)
        print("Status 2 :",status_2)
        if status_2 == SUCCEEDED:
            rospy.loginfo("he llegao a punto")


    if status == ABORTED or status_2 == ABORTED:
        print("ABORTEEED")
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)