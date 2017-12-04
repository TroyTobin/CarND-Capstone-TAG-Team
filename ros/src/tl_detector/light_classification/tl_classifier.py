from styx_msgs.msg import TrafficLight
import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import threading
import time

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image
from threading import Lock
from datetime import datetime


class TLClassifier(object):
    def __init__(self):

        # it takes time tu run the clasifyer so it is best to just return the
        # last known classification. Homework for later: bilt a faster model


        #initializing tensorflow
        dir_path = os.path.dirname(os.path.realpath(__file__))

        PATH_TO_MODEL = dir_path + '/graphs_22_faster_rcnn_bogdan_112900.pb'
        
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.FastGFile(PATH_TO_MODEL, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.sess = tf.Session(graph=self.detection_graph)

        #color codes have to match bosh_label_map used.pbtxt in training
        self.red_values = [2.0, 5.0, 6.0, 9.0, 13.0, 14.0]
        self.green_values = [1.0, 3.0, 4.0, 10.0, 11.0, 12.0]
        self.yellow_values = [7]

    def get_classification(self, image):

        with self.detection_graph.as_default():
            start_time = datetime.now().microsecond

            # # atempt to reduce computation for the clasifier
            # # it is abit faster this way but not suficient.
            # image = Image.fromarray(image,'RGB')
            # size = image.size
            # size = [0.5*x for x in size]
            # image.thumbnail(size,Image.ANTIALIAS)
            # image = image.getdata()
            # print('detecting ---', start_time)

            #Expand dimension since the model expects image to have shape [1, None, None, 3].
            img_expanded = np.expand_dims(image, axis=0)
            ( scores, classes ) = self.sess.run(
                [ self.d_scores, self.d_classes],
                feed_dict={self.image_tensor: img_expanded})

            end_time = datetime.now().microsecond
            classes = np.squeeze(classes)
            scores = np.squeeze(scores)
            #print(classes)
            #print (scores)
            color_val =  classes[0]
            score = scores[0]
            if color_val in self.red_values:
                print ("----> red   ", "Score = " , score, "<---", start_time, end_time)
                return TrafficLight.RED
            elif color_val in self.green_values:
                print ("----> green ", "Score = " , score, "<---", start_time, end_time)
                return TrafficLight.GREEN
            elif color_val in self.yellow_values:
                print ("----> yellow", "Score = " , score, "<---", start_time, end_time)
                return TrafficLight.YELLOW
            else:
                print ("----> unknown", "Score = " , score, "<---", start_time, end_time)
                return TrafficLight.UNKNOWN

    # def get_classification(self, image):
    #     thread = threading.Thread(target=self.clasify_in_background, args=(image,))
    #     thread.daemon = True
    #     thread.start()


# clasifier = TLClassifier()
# image = Image.open('image3.png')
#
# start_time = str(datetime.now())
# for i in range (4):
#     print ("Item: ", i)
#     clasifier.clasify_in_background(image)
# end_time = str(datetime.now())
# print ("Start time: ", start_time)
# print ("End time  : ", end_time)
