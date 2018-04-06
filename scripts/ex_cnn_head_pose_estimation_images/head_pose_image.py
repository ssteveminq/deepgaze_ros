#!/usr/bin/env python

#Copyright (c) 2016 Massimiliano Patacchiola
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
#MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
#CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
#SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
import sys
import roslib
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import select, termios, tty
from sensor_msgs.msg import Image
from std_msgs.msg import String
from os import listdir
import os
import tensorflow as tf
import cv2
import numpy as np
from deepgaze.head_pose_estimation import CnnHeadPoseEstimator

class Head_Dectectors(object):
    def __init__(self, wait=0.0):
        sess = tf.Session() #Launch the graph in a session.
        self.settings = termios.tcgetattr(sys.stdin)
        self.my_head_pose_estimator = CnnHeadPoseEstimator(sess) #Head pose estimation object
        self.roll_config_filepath = "/home/mk/hsr/attention_ws/src/interaction_functions/deepgaze_ros/etc/tensorflow/head_pose/roll/cnn_cccdd_30k.tf"
        self.pitch_config_filepath = "/home/mk/hsr/attention_ws/src/interaction_functions/deepgaze_ros/etc/tensorflow/head_pose/pitch/cnn_cccdd_30k.tf"
        self.yaw_config_filepath = "/home/mk/hsr/attention_ws/src/interaction_functions/deepgaze_ros/etc/tensorflow/head_pose/yaw/cnn_cccdd_30k"
        # Load the weights from the configuration folders
        self.my_head_pose_estimator.load_roll_variables(self.roll_config_filepath) 
        self.my_head_pose_estimator.load_pitch_variables(self.pitch_config_filepath) 
        self.my_head_pose_estimator.load_yaw_variables(self.yaw_config_filepath) 

        image_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
	rospy.Subscriber(image_topic, Image, self.image_callback)
        self.bridge = CvBridge()
        cv2.startWindowThread()
        # self.cv2_img = cv2.imread()
        # self.image = cv2.imread(file_name)
	
    def test_local_images(self): 
        for i in range(1,9):
            file_name = str(i) + ".jpg"
            print("Processing image ..... " + file_name)
            image = cv2.imread(file_name) #Read the image with OpenCV
           # Get the angles for roll, pitch and yaw
            roll = self.my_head_pose_estimator.return_roll(image)  # Evaluate the roll angle using a CNN
            pitch = self.my_head_pose_estimator.return_pitch(image)  # Evaluate the pitch angle using a CNN
            yaw = self.my_head_pose_estimator.return_yaw(image)  # Evaluate the yaw angle using a CNN
            print("Estimated [roll, pitch, yaw] ..... [" + str(roll[0,0,0]) + "," + str(pitch[0,0,0]) + "," + str(yaw[0,0,0])  + "]")
            print("")

    def image_callback(self,msg):
        # print('image_callback: ')
        # rospy.loginfo("image is of type: "+str(type(msg)))
        try:
            cv2_img   = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        # height, width, channels = self.cv2_img.shape
        # print height, width, channels 
            # cv2.startWindowThread()
            # cv2.imshow("image_window", cv2_img)
            # cv2.waitKey(30)
            resize_dim= (480,480)
            resized_img = cv2.resize(cv2_img,resize_dim,interpolation=cv2.INTER_AREA)
            cv2.imshow("image_window", resized_img)
            cv2.waitKey(30)
            # roll = self.my_head_pose_estimator.return_roll(cv2_img)  # Evaluate the roll angle using a CNN

        except CvBridgeError, e:
            print(e)

        roll = self.my_head_pose_estimator.return_roll(resized_img)  # Evaluate the roll angle using a CNN
        pitch = self.my_head_pose_estimator.return_pitch(resized_img)  # Evaluate the pitch angle using a CNN
        yaw = self.my_head_pose_estimator.return_yaw(resized_img)  # Evaluate the yaw angle using a CNN
        print("Estimated [roll, pitch, yaw] ..... [" + str(roll[0,0,0]) + "," + str(pitch[0,0,0]) + "," + str(yaw[0,0,0])  + "]")

    def listener(self,wait=0.0):
        rospy.spin() 

if __name__ == '__main__':
        rospy.init_node('head_orientation_estimation_test')
	# print("Initialize node")
        Head_manager = Head_Dectectors(sys.argv[1] if len(sys.argv) >1 else 0.0)
	Head_manager.listener()	

