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
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import geometry_msgs.msg
from os import listdir
import os
import tensorflow as tf
# import tf
# import tf2_ros
import cv2
import numpy as np
import matplotlib.pyplot as plt
import csv
from deepgaze.head_pose_estimation import CnnHeadPoseEstimator

class Head_Dectectors(object):
    def __init__(self, wait=0.0):
        sess = tf.Session() #Launch the graph in a session.
        self.settings = termios.tcgetattr(sys.stdin)
        self.my_head_pose_estimator = CnnHeadPoseEstimator(sess) #Head pose estimation object
        self.roll_config_filepath = "/home/mk/hsr/attention_ws/src/deepgaze_ros/etc/tensorflow/head_pose/roll/cnn_cccdd_30k.tf"
        self.pitch_config_filepath = "/home/mk/hsr/attention_ws/src/deepgaze_ros/etc/tensorflow/head_pose/pitch/cnn_cccdd_30k.tf"
        self.yaw_config_filepath = "/home/mk/hsr/attention_ws/src/deepgaze_ros/etc/tensorflow/head_pose/yaw/cnn_cccdd_30k"
        # Load the weights from the configuration folders
        self.my_head_pose_estimator.load_roll_variables(self.roll_config_filepath) 
        self.my_head_pose_estimator.load_pitch_variables(self.pitch_config_filepath) 
        self.my_head_pose_estimator.load_yaw_variables(self.yaw_config_filepath) 

        # image_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        image_topic = "/camera/rgb/image_rect_color"
	rospy.Subscriber(image_topic, Image, self.image_callback)
        self.bridge = CvBridge()
        cv2.startWindowThread()

        #tf frame pbulisher
        self.tfbuffer=tf2_ros.Buffer()
        self.tflistener=tf2_ros.TransformListener(self.tfbuffer)
        self.pose_pub = rospy.Publisher("/head_pose", PoseStamped,queue_size=1)

        self.my_list=[]

        # self.writer = csv.writer(open("/home/mk/attention_ws/src/deepgaze_ros/data.csv",'w'))
        # self.br=tf.TransformBroadcaster()
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
        # self.iter=self.iter+1;
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
            # cv2.waitKey(3)
            if cv2.waitKey(3) & 0xFF == ord('q'):
                # self.wfile.close()
                with open("test.csv",'w') as myfile:
                    writer=csv.writer(myfile,lineterminator='\n')
                    for rows in self.my_list:
                        writer.writerow(rows)
                    # writer.writerows(self.my_list)

                # self.rows=zip(self.rollvec,self.pitchvec,self.yawvec)
                # self.row_arr=np.array(self.rows)
                # self.row_arr=np.array(self.dataset)
                # np.savetxt("rpy_data.txt",self.row_arr)
                # return
                sys.exit()
                # break #Exit when Q is pressed

            # roll = self.my_head_pose_estimator.return_roll(cv2_img)  # Evaluate the roll angle using a CNN

        except CvBridgeError, e:
            print(e)

        roll = self.my_head_pose_estimator.return_roll(resized_img)  # Evaluate the roll angle using a CNN
        pitch = self.my_head_pose_estimator.return_pitch(resized_img)  # Evaluate the pitch angle using a CNN
        yaw = self.my_head_pose_estimator.return_yaw(resized_img)  # Evaluate the yaw angle using a CNN
        # roll = np.deg2rad(self.my_head_pose_estimator.return_roll(resized_img))  # Evaluate the roll angle using a CNN
        # pitch = np.deg2rad(self.my_head_pose_estimator.return_pitch(resized_img))  # Evaluate the pitch angle using a CNN
        # yaw = np.deg2rad(self.my_head_pose_estimator.return_yaw(resized_img))  # Evaluate the yaw angle using a CNN
        self.my_list.append([roll,pitch,yaw])

        roll_rad=np.deg2rad(roll)
        pitch_rad=np.deg2rad(pitch)
        yaw_rad=np.deg2rad(yaw)

        # self.rows=zip(roll,pitch,yaw)

        # self.row_arr=np.array(self.rows)
        # np.savetxt("rpy_data.txt",self.row_arr)
        # self.rollvec.append(roll)
        # self.pitchvec.append(pitch)
        # self.yawvec.append(yaw)
        print("Estimated [roll, pitch, yaw] ..... [" + str(roll[0,0,0]) + "," + str(pitch[0,0,0]) + "," + str(yaw[0,0,0])  + "]")
        quat = quaternion_from_euler(roll_rad,pitch_rad,yaw_rad)
        # print quat
        head_frame = PoseStamped()
        head_frame.header.frame_id = "human"
        head_frame.header.stamp=rospy.Time.now()
        # head_frame.child_frame_id = "human_head"
        head_frame.pose.position.x=0.0
        head_frame.pose.position.y=0.0
        head_frame.pose.position.z=0.0
        head_frame.pose.orientation.x=quat[0]
        head_frame.pose.orientation.y=quat[1]
        head_frame.pose.orientation.z=quat[2]
        head_frame.pose.orientation.w=quat[3]
        # tfm=TFMessage([head_frame])
        self.pose_pub.publish(head_frame)
        # self.iter=self.iter+1;
        

            # if cv2.waitKey(3) & 0xFF == ord('q'):
        # br.sendTransform((0.0, 2.0, 3.0), (0.0, 0.0, 0.0, 0.1), rospy.Time.now(),"world","human")

    def listener(self,wait=0.0):
        rospy.spin() 

if __name__ == '__main__':
        rospy.init_node('head_orientation_estimation_test')
	# print("Initialize node")
        Head_manager = Head_Dectectors(sys.argv[1] if len(sys.argv) >1 else 0.0)
	Head_manager.listener()	
        

