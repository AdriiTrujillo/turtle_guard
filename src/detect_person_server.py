#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
import actionlib
import numpy as np
import tensorflow as tf
import cv2
import time
import sys

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Importamos los mensajes de este Action
from turtle_guard.msg import GuardAction
from turtle_guard.msg import GuardGoal
from turtle_guard.msg import GuardResult
from turtle_guard.msg import GuardFeedback


# Clase que corre la red neuronal 
class DetectorAPI:
    # En el constructor realizamos las inicializaciones necesarias
    def __init__(self, path_to_ckpt):
        self.path_to_ckpt = path_to_ckpt

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.compat.v1.GraphDef()
            with tf.compat.v2.io.gfile.GFile(self.path_to_ckpt, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.default_graph = self.detection_graph.as_default()
        self.sess = tf.compat.v1.Session(graph=self.detection_graph)

        # Define los Tonsores de entrada y salida para detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Cada box representa una parte de la imagen donde un objeto en partiuclar fue detectado.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Cada puntuacion representa el nivel de como de confiable es cada objeto.
        # La puntuacion se muestra en la imagen resultado junto con la etiqueta de la clase.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    # Metodo para realizar el procesamiento de la imagen 
    def processFrame(self, image):

        threshold = 0.7
        # Expande las dimensiones ya que el modelo entrenado espera imagenes que tengan shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image, axis=0)

        # Proceso de deteccion
        start_time = time.time()
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})
        end_time = time.time()

        print("Elapsed Time:", end_time-start_time)

        # Procesamiento para generar los boxes de cada objeto detectado.
        im_height, im_width,_ = image.shape
        boxes_list = [None for i in range(boxes.shape[1])]
        for i in range(boxes.shape[1]):
            boxes_list[i] = (int(boxes[0,i,0] * im_height),
                        int(boxes[0,i,1]*im_width),
                        int(boxes[0,i,2] * im_height),
                        int(boxes[0,i,3]*im_width))

        # Inicializaciones
        boxes_f = boxes_list
        scores_f = scores[0].tolist()
        classes_f = [int(x) for x in classes[0].tolist()]

        cont = 0

        # Filtro para descartar falsos positivos
        for i in range(len(boxes_f)):
            # Clase 1 representa humanos
            if classes_f[i] == 1 and scores_f[i] > threshold:
                cont += 1

        persona = False

        # Si se ha encotrado algún humano cambiamos el booleano a True
        if cont > 0:
            persona = True

        return persona

    def close(self):
        self.sess.close()
        self.default_graph.close()

# --------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------

# Clase para obtener procesar fotos de la camara 
class TakePhoto:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Topic de la camara
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Esperar 1 segundo para conectar el topic
        rospy.sleep(1)

    def callback(self, data):

        # Convertir la imagen a OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        # Guardar la imagen en la variable de la clase
        self.image = cv_image

    # Metodo para guardar la imagen en el equipo
    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False

    # Metodo para devolver la imagen 
    def get_image(self):
        if self.image_received:
            return self.image

# --------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------

# Clase que contiene el Action server completo
class detect_Person(object):
    # create messages that are used to publish feedback/result
    _feedback = GuardFeedback()
    _result = GuardResult()
    # CAMBIAR SEGUN LA RUTA DEL EQUIPO QUE SE USE
    _model_path = '/home/adrii/catkin_ws/src/turtle_guard/Coco_Model/faster_rcnn_inception_v2_coco_2018_01_28/frozen_inference_graph.pb'
    _odapi = DetectorAPI(path_to_ckpt=_model_path)
    _camera = TakePhoto()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GuardAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        # CAMBIAR SEGUN LA RUTA DEL EQUIPO QUE SE USE
        self._path = '/home/adrii/catkin_ws/src/turtle_guard/pictures/intruso.jpg'
    
    # Función callback donde se realiza el procesamiento
    def execute_cb(self, goal):

        # Inicializaciones
        r = rospy.Rate(1)
        success = True
        detected = False
    
        # Cambiamos el estado del feedback
        self._feedback.start = True
        
        rospy.loginfo("Detectando persona ...")

        # Creamos una variable y guardamos una imagen de la camara
        img = self._camera.get_image()

        while not detected:

            # Actualizamos la imagen en cada iteracion
            img = self._camera.get_image()
            # Cambiamos las medidas para que la red pueda trabajar mejor
            img = cv2.resize(img, (1280, 720))
            # Obtenemos el resultado de la red neuronal para la imagen en esta iteracion
            detected = self._odapi.processFrame(img)

            # Se comprueba que el cliente no ha cancelado el action service
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False # Si se cancela el actión no se completa el proceso
                break

            # Publciar un feedback como que sigue activo el action service
            self._as.publish_feedback(self._feedback)
            r.sleep()

        # Si se ha terminado el proceso sin cancelarse
        if success:
            # El action service devuelve True
            self._result.person_detected = True 
            # Se guarda la imagen en el equipo
            cv2.imwrite(self._path, img)
            rospy.loginfo("Saved image at " + self._path)
            rospy.loginfo('%s: Succeeded' % self._action_name)
            # Se camibia el estado del action service a completado
            self._as.set_succeeded(self._result)

# En el main se crea un objeto de la clase detect_person
# y se hace rosoy.spin() para que el servicio continue de manera continua
if __name__ == '__main__':
    rospy.init_node('detect_person_action_server')
    server = detect_Person("detect_person_action")
    rospy.spin()