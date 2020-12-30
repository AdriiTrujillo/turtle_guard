#! /usr/bin/env python

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
from turtle_guard.msg import GuardAction
from turtle_guard.msg import GuardGoal
from turtle_guard.msg import GuardResult
from turtle_guard.msg import GuardFeedback



class DetectorAPI:
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

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def processFrame(self, image):
        threshold = 0.7
        # Expand dimensions since the trained_model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image, axis=0)
        # Actual detection.
        start_time = time.time()
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})
        end_time = time.time()

        print("Elapsed Time:", end_time-start_time)

        im_height, im_width,_ = image.shape
        boxes_list = [None for i in range(boxes.shape[1])]
        for i in range(boxes.shape[1]):
            boxes_list[i] = (int(boxes[0,i,0] * im_height),
                        int(boxes[0,i,1]*im_width),
                        int(boxes[0,i,2] * im_height),
                        int(boxes[0,i,3]*im_width))


        boxes_f = boxes_list
        scores_f = scores[0].tolist()
        classes_f = [int(x) for x in classes[0].tolist()]

        cont = 0

        for i in range(len(boxes_f)):
            # Class 1 represents human
            if classes_f[i] == 1 and scores_f[i] > threshold:
                cont += 1

        persona = False

        if cont > 0:
            persona = True

        return persona

    def close(self):
        self.sess.close()
        self.default_graph.close()

# --------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------

class TakePhoto:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False

    def get_image(self):
        if self.image_received:
            return self.image

# --------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------

class detect_Person(object):
    # create messages that are used to publish feedback/result
    _feedback = GuardFeedback()
    _result = GuardResult()
    _model_path = '/home/adrii/catkin_ws/src/turtle_guard/Coco_Model/faster_rcnn_inception_v2_coco_2018_01_28/frozen_inference_graph.pb'
    _odapi = DetectorAPI(path_to_ckpt=_model_path)
    _camera = TakePhoto()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GuardAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        detected = False
    
        # append the seeds for the fibonacci sequence
        self._feedback.start = True
        
        rospy.loginfo("Detectando persona")
        # self._camera.take_picture("picture.jpg")
        # cv2.imshow('image',self._camera.get_image())

        while not detected:

            img = self._camera.get_image()
            img = cv2.resize(img, (1280, 720))
            detected = self._odapi.processFrame(img)

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()

        cv2.destroyWindow('image')

        if success:
            self._result.person_detected = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('detect_person_action_server')
    server = detect_Person("detect_person_action")
    rospy.spin()