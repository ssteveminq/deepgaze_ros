#!/usr/bin/python
#!/usr/bin/env python

# ROS version written by Minkyu Kim

#The MIT License (MIT)
#Copyright (c) 2016 Massimiliano Patacchiola
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
#MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
#CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
#SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#In this example the Particle Filter is used in order to stabilise some noisy detection.
#The Backprojection algorithm is used in order to find the pixels that have the same HSV 
#histogram of a predefined template. The template is a subframe of the main image or an external
#matrix that can be used as a filter. We track the object taking the contour with the largest area
#returned by a binary mask (blue rectangle). The center of the contour is the tracked point. 
#To test the Particle Filter we inject noise in the measurements returned by the Backprojection. 
#The Filter can absorbe the noisy measurements, giving a stable estimation of the target center (green dot).

#COLOR CODE:
#BLUE: the rectangle containing the target. Noise makes it shaky (unstable measurement).
#GREEN: the point estimated from the Particle Filter.
#RED: the particles generated by the filter.

import roslib;
import rospy
import sys
import time
# print(sys.path)
# sys.path.remove('/home/mk/anaconda3/envs/py27/lib/python2.7/site-packages')
import cv2
# sys.path.add('/home/mk/anaconda3/envs/py27/lib/python2.7/site-packages')
print(cv2.__version__)
import numpy as np
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import select, termios, tty
from sensor_msgs.msg import Image
from std_msgs.msg import String
from os import listdir
import os
# import tensorflow as tf

ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--image", required=True,
	# help="path to input image")
ap.add_argument("-y", "--yolo", default="yolo-coco",
	help="base path to YOLO directory")
ap.add_argument("-c", "--confidence", type=float, default=0.6,
	help="minimum probability to filter weak detections")
ap.add_argument("-t", "--threshold", type=float, default=0.3,
	help="threshold when applyong non-maxima suppression")
args = vars(ap.parse_args())
# print("hmm")

# labelsPath = os.path.sep.join([args["yolo"], "coco.names"])
labelsPath='/home/mk/workspaces/yolo-object-detection/yolo-coco/coco.names'
# print(labelsPath)
LABELS = open(labelsPath).read().strip().split("\n")

np.random.seed(42)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
	dtype="uint8")

weightsPath = os.path.sep.join([args["yolo"], "yolov3.weights"])
configPath = os.path.sep.join([args["yolo"], "yolov3.cfg"])

weightsPath='/home/mk/workspaces/yolo-object-detection/yolo-coco/yolov3.weights'
configPath='/home/mk/workspaces/yolo-object-detection/yolo-coco/yolov3.cfg'

net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

# print("hmm2")

# from deepgaze.color_detection import BackProjectionColorDetector
# from deepgaze.mask_analysis import BinaryMaskAnalyser
# from deepgaze.motion_tracking import ParticleFilter

class ObjectTracker(object):
    def __init__(self, wait=0.0):

        self.bridge = CvBridge()
        # image_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        # image_topic = "/hsrb/head_rgbd_sensor/rgb/image_color"
        # image_topic = '/head_mount_kinect2/rgb/image_raw'
        image_topic = '/rgbd_sensor/rgb/image_raw'
	rospy.Subscriber(image_topic, Image, self.image_callback)
        # rospy.spin()

    def image_callback(self,msg):
        print('image_callback')
        # print(msg)
        # sleep(0.1)
        boxes = []
        confidences = []
        classIDs = []
        # frame=[]
 
        # cv_img= self.bridge.imgmsg_to_cv2(msg,"bgr8")

        try:
            # print("hello")
            # frame= self.bridge.imgmsg_to_cv2(msg)
            # cv_img= self.bridge.imgmsg_to_cv2(msg,"passthrough")
            frame   = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        except CvBridgeError, e:
            # print("hello2")
            print(e)
        # roslogprint("get frames")
        #yolo
        
        (H, W) = frame.shape[:2]
        ln = net.getLayerNames()
        ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416),
                    swapRB=True, crop=False)
        net.setInput(blob)
        start = time.time()
        layerOutputs = net.forward(ln)
        end = time.time()

        for output in layerOutputs:
                # print(len(output))
                # input("enter to continue")
                # loop over each of the detections
            for detection in output:
                    # extract the class ID and confidence (i.e., probability) of
                    # the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                    
            # filter out weak predictions by ensuring the detected
            # probability is greater than the minimum probability
                if confidence > args["confidence"]:
                # if confidence > 0.05:
                    # print(LABELS[classID]+"detected!!")
                    # scale the bounding box coordinates back relative to the
                    # size of the image, keeping in mind that YOLO actually
                    # returns the center (x, y)-coordinates of the bounding
                    # box followed by the boxes' width and height
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    # self.sample_trj[LABELS[classID]].append([centerX,centerY])


                    # use the center (x, y)-coordinates to derive the top and
                    # and left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    # update our list of bounding box coordinates, confidences,
                    # and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

                    idxs = cv2.dnn.NMSBoxes(boxes, confidences, args["confidence"],
                            args["threshold"])

                    if len(idxs) > 0:
                        # loop over the indexes we are keeping
                        for i in idxs.flatten():
                            # extract the bounding box coordinates
                            (x, y) = (boxes[i][0], boxes[i][1])
                            (w, h) = (boxes[i][2], boxes[i][3])

                            # draw a bounding box rectangle and label on the image
                            color = [int(c) for c in COLORS[classIDs[i]]]
                            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                            text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
                            cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, color, 2)

        
        cv2.imshow("image_window", frame)
        # cv2.imshow('Mask', frame_mask) #show on window
        cv2.waitKey(1)
            # if cv2.waitKey(3) & 0xFF == ord('q'):
                # break #Exit when Q is pressed

    def listener(self,wait=0.0):
        # while not rospy.is_shutdown():
        rospy.spin() 


if __name__ == '__main__':
        rospy.init_node('particle_filter_test')
        print("Initialize node")
        tracker_manager = ObjectTracker()
        tracker_manager.listener()
