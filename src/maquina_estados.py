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
# Mensajes del action para detectar personas
from turtle_guard.msg import GuardAction, GuardGoal, GuardResult
import tf

point = int(0)

# Poisbles estados de un action server según el 
# numero que devuelven 

PENDING = 0
ACTIVE = 1
PREEMPTED = 2
SUCCEEDED = 3
ABORTED = 4
RECALLED = 8

# Puntos guardados donde debe ir a vigilar el turtlebot
waypoints = [
	[(-3.6424, -3.4765, 0.0000), (0.0000, 0.0000, 0.2121, 0.9772)],
	[(1.1133, -2.5329, 0.0000), (0.0000, 0.0000, 0.5976, 0.8018)],
	[(3.2502, 2.3253, 0.0000),(0.0000, 0.0000, 0.9831, 0.1829)],
	[(-1.1888, 3.8447, 0.0000), (0.0000, 0.0000, -0.5554, 0.8316)],
	[(3.3542, 7.1520, 0.0000),(0.0000, 0.0000, 0.9933, 0.1553)],
	[(-2.4514, 7.4468, 0.0000, 0.0000),(0.0000, 0.0000, 0.1949, 0.9808)]
]

# Funcion para crear el mensaje que se envia para mover la base del robot
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

# Funcion para crear el menaje que se envia a la base para que gire un angulo dado
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

# Estado en el que el robot va a un punto dado
class IrAPunto(smach.State):
    """Estado que lleva el robot a los diferentes puntos del mapa"""

    def __init__(self):
        # Posibles salidas del estado
        smach.State.__init__(self, outcomes=['fallo','llega_a_punto', 'persona_detectada'])
        self.counter = 0

    def execute(self, userdata):
        # Inicializaciones
        global point
        detection_result = True
        movement_result = False

        # Action client para la base del robot 
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo('Yendo a punto ...')
        # Action client para detectar personas
        client_p = actionlib.SimpleActionClient('detect_person_action', GuardAction)
        client_p.wait_for_server()
        rospy.loginfo('Buscando intruso ...')
        
        # Crear el mensaje para enviar a la base del robot
        point_goal = goal_pose(waypoints[point])
        # Crear el mensaje para enviar al action de detectar personas
        detection_goal = GuardGoal()
        detection_goal.finish = True
        # Se envian los dos mensajes
        client.send_goal(point_goal)
        client_p.send_goal(detection_goal)

        # Obtenemos el estado del action de movimiento 
        movement_status = client.get_state()
        # Obtenemos el estado del action para detectar personas
        detection_status = client_p.get_state()

        # Mientras que nignun estado se considere como completado seguir comprobando los estados
        while movement_status < SUCCEEDED and detection_status < SUCCEEDED:
            movement_status = client.get_state()
            detection_status = client_p.get_state()

        # Si el robot llega al punto --> Cancelamos la deteccion de personas
        if movement_status == SUCCEEDED and detection_status < SUCCEEDED:
            client_p.cancel_goal()

        # Si el robot detecta alguna persona --> Cancelamos el movimiento del robot al punto
        if detection_status == SUCCEEDED and movement_status < SUCCEEDED:
            client.cancel_goal()

        # Si se cancela internamente por error la deteccion de personas
        if detection_status == RECALLED:
            rospy.loginfo("Detección cancelada")

        # Si se cancela internamente por error el movimiento de la base
        if movement_status == RECALLED:
            rospy.loginfo("Movimiento cancelado")

        # Obtenemos el resultado de la detección de personas
        detection_result = client_p.get_result()
        # Esperamos un segundo para que de tiempo a procesar
        rospy.sleep(1.0)
        # Obtenemos el resultado del movimiento de la base
        movement_result = client.get_result() 

        # Si se ha detectado una persona
        if detection_result:
            return 'persona_detectada'
        # Si el movimiento se ha completado correctamente
        elif movement_result:
            return 'llega_a_punto'
        else:
            return 'fallo'

# Estado en el que robot gira sobre si mismo y comprueba que no hay ningún intruso
class Girar(smach.State):
    """Estado que gira el robot sobre si mismo mientras comprueba que no hay intrusos"""
    def __init__(self):
        # Posibles salidas del estado
        smach.State.__init__(self, outcomes=['fallo','giro_terminado', 'persona_detectada'])
        self.counter = 0

    def execute(self, userdata):
        # Inicializaciones
        global point
        detection_result = True
        turn_result = False

        # Action client para la base del robot 
        client_g = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client_g.wait_for_server()
        rospy.loginfo('Girando ... ')
        # Action client para detectar personas
        client_p = actionlib.SimpleActionClient('detect_person_action', GuardAction)
        client_p.wait_for_server()
        rospy.loginfo('Buscando intrusos ...')     

        # Creamos el mensaje para comenzar a detectar personas
        detection_goal = GuardGoal()
        detection_goal.finish = True
        # Enviamos el mensaje al cliente para comenzar la deteccion
        client_p.send_goal(detection_goal)
        # Obtenemos el estado de la deteccion (Inicializacion)
        detection_status = client_p.get_state()   

        # For para realizar el giro sobre si mismo en cuatro giros de 90°
        for i in range(5):
            
            # Convertimos los grados a quaternios para poder crear el mensaje para la base necesario
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, (math.pi/2)*i) 
            # Creamos el mensaje
            point_goal = giro(waypoints[point], quaternion)
            # Enviamos el mensaje a la base
            client_g.send_goal(point_goal)
            # Obtenemos el estado del giro de la base (Inicializacion)
            turn_status = client_g.get_state()
            # Obtenemos el estado de la deteccion de personas (Inicializacion)
            detection_status = client_p.get_state()
            # Mientras que el giro de la base y la deteccion de personas no termine se comprueba su estado continuamente
            while turn_status < SUCCEEDED and detection_status < SUCCEEDED: 
                turn_status = client_g.get_state()
                detection_status = client_p.get_state()

            # Si se detecta que hay una persona durante el giro se cancela el action del giro
            if detection_status == SUCCEEDED and turn_status < SUCCEEDED:
                client_g.cancel_goal()
                # Se sale del bucle y se deja de girar aunque no se ha dado una vuelta entera
                break
            
            # Si se cancela internamente por error la deteccion de personas
            if detection_status == RECALLED:
                rospy.loginfo("Detección cancelada")
            # Si se cancela internamente por error el giro de la base
            if turn_status == RECALLED:
                rospy.loginfo("Giro cancelado")

        # Cuando se ha terminado de realizar todo el giro se comprueba que 
        # si se ha completado correctamente el giro sin que se detecte nignun 
        # intruso se cancela la deteccion de personas.  
        if turn_status == SUCCEEDED and detection_status < SUCCEEDED:
            client_p.cancel_goal()

        # Se obtiene el resultado de la deteccion de personas
        detection_result = client_p.get_result()
        # Se espera un segundo para dar tiempo a procesar datos
        rospy.sleep(1.0)
        # Se obtiene el resultado del giro
        turn_result = client_g.get_result() 

        # Si no se ha llegado al ultimo giro que se ha declarado
        if point < len(waypoints)-1:
            # Se utiliza el siguiente punto al que se esta utilizando
            point += 1
        # Si se ha llegado al ultimo punto que se ha declarado
        else:
            # Se vuelve a empezar desde el primer punto declarado   
            point = 0

        # Si se ha detectado una persona
        if detection_result:
            return 'persona_detectada'
        # Si se ha completado del giro correctamente
        elif turn_result:
            return 'giro_terminado'
        else:
            return 'fallo'

class Alarma(smach.State):
    """Estado de Alarma que sucede cuando se encuentre una persona."""

    def __init__(self):
        # Posibles salidas del estado
        smach.State.__init__(self, outcomes=['fallo','terminado'])
        self.counter = 0

    def execute(self, userdata):
        # Solamente se da informacion al usuario
        rospy.loginfo(" ¡¡ Hay Alguien !! ")
        rospy.loginfo(" Se ha detectado un intruso ... ")
        rospy.loginfo(" Avisando a las autoridades locales ... ")
        # Desde el action diseñado a nivel interno siempre que se detecta una persona 
        # se guarda una foto del frame en el que se ha detectado una persona 
        rospy.loginfo(" Compruebe su directorio configurado para ver la imagen obtenida ... ")

        # Despues de este estado siempre se termina el ciclo de la maquina de estados
        if True:
            return 'terminado'
        else:
            return 'fallo'
                    
def main():

    # Se crea la máquina de estados SMACH con sus posibles salidas
    sm = smach.StateMachine(outcomes=['FALLO', 'OK'])

    with sm:
        # Se añaden los estados a la máquina de estados, así como sus relaciones
        smach.StateMachine.add('IrAPunto', IrAPunto(), 
                                transitions={'fallo':'FALLO', 
                                             'llega_a_punto':'Girar', # Si se llega a un punto --> girar sobre si mismo
                                             'persona_detectada': 'Alarma'}) # Si detecta alguien --> enviar una alarma
        smach.StateMachine.add('Girar', Girar(), 
                                transitions={'fallo':'FALLO', 
                                             'giro_terminado':'IrAPunto', # Si termina de girar --> ves al siguiente punto
                                             'persona_detectada': 'Alarma'}) # Si detecta a alguien --> enviar una alarma
        # Despues de la alarma siempre se temrmina el ciclo
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

# Main del programa
if __name__ == '__main__':
    main()  