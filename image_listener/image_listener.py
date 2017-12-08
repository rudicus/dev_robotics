
#%matplotlib inline
import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import glob

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt

import PIL
import json


# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import String
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# This is needed to display the images.

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")

import threading
from utils import label_map_util

from utils import visualization_utils as vis_util
import random

class tensor_detector:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_topic = "/io/internal_camera/right_hand_camera/image_raw"
        self.sub = rospy.Subscriber(self.image_topic, Image,self.callback, queue_size=10)

        self.pub = rospy.Publisher('affordances', String, queue_size=10)

        self.PATH_TO_CKPT = '/data/users/mryan/src/intera/src/tensor/graphs/class_graph.pb/frozen_inference_graph.pb'
        self.PATH_TO_LABELS = '/data/users/mryan/src/intera/src/tensor/graphs/class_graph.pb/labels.pbtext'
        self.NUM_CLASSES = 5

        self.detection_graph = tf.Graph()

        self.label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
        self.categories = label_map_util.convert_label_map_to_categories(self.label_map, max_num_classes=self.NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(self.categories)

        self.latest_image = []
        self.imageLock = threading.Lock()
        self.condition = threading.Condition()

        self.shuttingDown = False

        self.imagePath = "./img.jpg"

        #load up the graph
        with self.detection_graph.as_default():
            self.od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                self.od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(self.od_graph_def, name='')


    def callback(self,msg):
        #print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.condition.acquire()
            cv2.imwrite( self.imagePath, cv2_img )
            self.condition.notify()
            self.condition.release()
            #print("Converted! an image!")
        except CvBridgeError, e:
            print(e)


    def load_image_into_numpy_array(self,image):
      (im_width, im_height) = image.size
      print image.size
      return np.array(image.getdata()).reshape(
          (im_height, im_width, 3)).astype(np.uint8)

    def start_thread(self):
        self.thread = threading.Thread(target=self.run_detection_loop)
        self.thread.start()

    def wait(self):
        self.shuttingDown = True
        self.thread.join()

    def run_detection_loop(self):
        IMAGE_SIZE = (12, 8)
        image = [];
        with self.detection_graph.as_default():
          with tf.Session(graph=self.detection_graph) as sess:
            # Definite input and output Tensors for detection_graph
            image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
            fig = plt.figure(figsize=IMAGE_SIZE)
            fig.show()

            #wait for callback data
            while not self.shuttingDown:
                self.condition.acquire()
                self.condition.wait()
                image = PIL.Image.open(self.imagePath)
                self.condition.release()

                print("Running Tensor!!")
                # the array based representation of the image will be used later in order to prepare the
                # result image with boxes and labels on it.
                image_np = self.load_image_into_numpy_array(image)
                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image_np, axis=0)
                # Actual detection.
                (boxes, scores, classes, num) = sess.run(
                    [detection_boxes, detection_scores, detection_classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})
                 # Visualization of the results of a detection.

                max_v = 0.0
                #print(np.shape(scores))
                #print(np.shape(boxes))
                #print(np.shape(classes))
                mscores = scores[0]
                mboxes =  boxes[0]
                mclasses = classes[0]

                detections = []
                thresh = .60;
                for i in range(len(mscores)):
                    if mscores[i] > thresh:
                        det = {}
                        det['score'] = np.asscalar(mscores[i])
                        det['class'] = int(np.asscalar(mclasses[i]))
                        det['box'] = {}
                        det['box']['ymin'] = np.asscalar(mboxes[i][0])
                        det['box']['xmin'] = np.asscalar(mboxes[i][1])
                        det['box']['ymax'] = np.asscalar(mboxes[i][2])
                        det['box']['xmax'] = np.asscalar(mboxes[i][3])
                        detections.append(det)

                    #    max_v = percent
                affordanceString = json.dumps(detections)
                self.pub.publish(affordanceString)

                #print(np.squeeze(boxes))
                #print(np.squeeze(scores))
                #print(np.squeeze(classes))
                vis_util.visualize_boxes_and_labels_on_image_array(
                  image_np,
                  np.squeeze(boxes),
                  np.squeeze(classes).astype(np.int32),
                  np.squeeze(scores),
                  self.category_index,
                  use_normalized_coordinates=True,
                  line_thickness=8)
                plt.clf()
                plt.imshow(image_np)
                #plt.suptitle(image_path)
                fig.canvas.draw()

def main():
    tc = tensor_detector();
    tc.start_thread()
    rospy.init_node('image_listener')
    # Define your image topic
    # Set up your subscriber and define its callback
    #sub = rospy.Subscriber(image_topic, Image, image_callback, queue_size=10)
    # Spin until ctrl + c
    rospy.spin()
    tc.wait()
    print("Exiting!")

if __name__ == '__main__':
    main()
