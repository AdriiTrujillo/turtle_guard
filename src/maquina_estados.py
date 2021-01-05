#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import numpy as np
import sys
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from turtle_guard.msg import GuardAction, GuardGoal, GuardResult
import tf

point = int(0)
intruso = False

PENDING = 0
ACTIVE = 1
PREEMPTED = 2
SUCCEEDED = 3
ABORTED = 4
RECALLED = 8

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

rospy.init_node('turtle_guard')

class IrAPunto(smach.State):
    """Estado que lleva el robot a los diferentes puntos del mapa"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['fallo','llega_a_punto', 'persona_detectada'])
        self.counter = 0

    def execute(self, userdata):
        global point
        detection_result = True
        movement_result = False

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo('Yendo a punto ...')
        client_p = actionlib.SimpleActionClient('detect_person_action', GuardAction)
        client_p.wait_for_server()
        rospy.loginfo('Buscando intruso ...')
        
        point_goal = goal_pose(waypoints[point])
        detection_goal = GuardGoal()
        detection_goal.finish = True
        client.send_goal(point_goal)
        client_p.send_goal(detection_goal)

        movement_status = client.get_state()
        detection_status = client_p.get_state()

        while movement_status < SUCCEEDED and detection_status < SUCCEEDED:
            movement_status = client.get_state()
            detection_status = client_p.get_state()

        if movement_status == SUCCEEDED and detection_status < SUCCEEDED:
            client_p.cancel_goal()

        if detection_status == SUCCEEDED and movement_status < SUCCEEDED:
            client.cancel_goal()

        if detection_status == RECALLED:
            rospy.loginfo("Detección cancelada")

        if movement_status == RECALLED:
            rospy.loginfo("Movimiento cancelado")

        detection_result = client_p.get_result()
        rospy.sleep(1.0)
        movement_result = client.get_result() 

        if detection_result:
            return 'persona_detectada'
        elif movement_result:
            return 'llega_a_punto'
        else:
            return 'fallo'

class Girar(smach.State):
    """Estado que lleva gira el robot sobre si mismo"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['fallo','giro_terminado', 'persona_detectada'])
        self.counter = 0

    def execute(self, userdata):
        global point
        detection_result = True
        turn_result = False

        client_g = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client_g.wait_for_server()
        rospy.loginfo('Girando ... ')
        client_p = actionlib.SimpleActionClient('detect_person_action', GuardAction)
        client_p.wait_for_server()
        rospy.loginfo('Buscando intrusos ...')     

        detection_goal = GuardGoal()
        detection_goal.finish = True
        client_p.send_goal(detection_goal)
        detection_status = client_p.get_state()   

        for i in range(5):

            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, (math.pi/2)*i) 
            point_goal = giro(waypoints[point], quaternion)

            client_g.send_goal(point_goal)
            
            turn_status = client_g.get_state()
            detection_status = client_p.get_state()

            while turn_status < SUCCEEDED and detection_status < SUCCEEDED: 
                turn_status = client_g.get_state()
                detection_status = client_p.get_state()

            if detection_status == SUCCEEDED and turn_status < SUCCEEDED:
                client_g.cancel_goal()
                break

            if detection_status == RECALLED:
                rospy.loginfo("Detección cancelada")

            if turn_status == RECALLED:
                rospy.loginfo("Giro cancelado")

        if turn_status == SUCCEEDED and detection_status < SUCCEEDED:
            client_p.cancel_goal()

        detection_result = client_p.get_result()
        rospy.sleep(1.0)
        turn_result = client_g.get_result() 

        if point < len(waypoints)-1:
            point += 1
        else:
            # detection_result = True
            point = 0

        if detection_result:
            return 'persona_detectada'
        elif turn_result:
            return 'giro_terminado'
        else:
            return 'fallo'

class Alarma(smach.State):
    """Estado de Alarma."""

    def __init__(self):
        smach.State.__init__(self, outcomes=['fallo','terminado'])
        self.counter = 0

    def execute(self, userdata):

        rospy.loginfo(" ¡¡ Hay Alguien !! ")
        rospy.loginfo(" Se ha detectado un intruso ... ")
        rospy.loginfo(" Avisando a las autoridades locales ... ")
        rospy.loginfo(" Compruebe su directorio configurado para ver la imagen obtenida ... ")

        if True:
            return 'terminado'
        else:
            return 'fallo'
                    
def main():

    # Se crea la máquina de estados SMACH
    sm = smach.StateMachine(outcomes=['FALLO', 'OK'])

    with sm:
        # Se añaden los estados a la máquina de estados, así como sus relaciones
        smach.StateMachine.add('IrAPunto', IrAPunto(), 
                                transitions={'fallo':'FALLO', 
                                             'llega_a_punto':'Girar',
                                             'persona_detectada': 'Alarma'})
        smach.StateMachine.add('Girar', Girar(), 
                                transitions={'fallo':'FALLO', 
                                             'giro_terminado':'IrAPunto',
                                             'persona_detectada': 'Alarma'})
        smach.StateMachine.add('Alarma', Alarma(), 
                                transitions={'fallo':'FALLO', 
                                             'terminado':'OK'})
            

    # Se crea el servidor para visualizar la máquina de estados
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Se ejecuta el plan SMACH
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()  