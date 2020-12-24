#!/usr/bin/env python

import rospy
import actionlib
from math import pi
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Positions to check ---------------------------------------

# (-3.6424, -3.4765, 0.0000, 0.0000, 0.0000, 0.2121, 0.9772)
# (1.1133, -2.5329, 0.0000, 0.0000, 0.0000, 0.5976, 0.8018)
# (3.2502, 2.3253, 0.0000, 0.0000, 0.0000, 0.9831, 0.1829)
# (-1.1888, 3.8447, 0.0000, 0.0000, 0.0000, -0.5554, 0.8316)
# (3.3542, 7.1520, 0.0000, 0.0000, 0.0000, 0.9933, 0.1553)
# (-2.4514, 7.4468, 0.0000, 0.0000, 0.0000, 0.1949, 0.9808)

# -----------------------------------------------------------

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

def giro(pose, quaternion):
	goal_pose = MoveBaseGoal()	
	goal_pose.target_pose.header.frame_id = 'map'
	goal_pose.target_pose.pose.position.x = pose[0][0]
	goal_pose.target_pose.pose.position.y = pose[0][1]
	goal_pose.target_pose.pose.position.z = pose[0][2]
	goal_pose.target_pose.pose.orientation.x = quaternion[0]
	goal_pose.target_pose.pose.orientation.y = quaternion[1]
	goal_pose.target_pose.pose.orientation.z = quaternion[2]
	goal_pose.target_pose.pose.orientation.w = quaternion[3]

	return goal_pose

if __name__ == '__main__':
	rospy.init_node('patrol')
	
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	
	while not rospy.is_shutdown():
		
		rospy.loginfo('Comenzando patrulla normal ... ')
		for pose in waypoints:
			goal = goal_pose(pose)
			client.send_goal(goal)
			client.wait_for_result()

			
			# Opcion 1
			rospy.loginfo('Buscando intrusos ...')
			for i in range(5):
				quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 1.57*i) 
				goal = giro(pose, quaternion)
				client.send_goal(goal)
				client.wait_for_result()

			# # Opcion 2
			# pose = goal + 180
			# goal = goal_pose(pose)
			# client.send_goal(goal)
			# client.wait_for_result()
		
		rospy.loginfo('Comenzando patrulla en orden inverso ... ')
		for pose in reversed(waypoints):
			
			goal = goal_pose(pose)
			client.send_goal(goal)
			client.wait_for_result()

			rospy.loginfo('Buscando intrusos ...')
			for i in range(4):
				quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 1.57*i) 
				goal = giro(pose, quaternion)
				client.send_goal(goal)
				client.wait_for_result()
		
