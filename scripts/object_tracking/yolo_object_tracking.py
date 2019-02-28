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

import cv2
import roslib
import sys
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import select, termios, tty
from sensor_msgs.msg import Image
from std_msgs.msg import String
from os import listdir
import os
from os.path import isfile, join, exists
import tensorflow as tf

from std_msgs.msg import Float32MultiArray
from deepgaze.color_detection import BackProjectionColorDetector
from deepgaze.mask_analysis import BinaryMaskAnalyser
from deepgaze.motion_tracking import ParticleFilter
from deepgaze_ros.srv import learn_clothes, learn_clothesRequest, learn_clothesResponse
from villa_yolocloud.msg import BboxInfo
from villa_yolocloud.msg import ObjectInfo
from villa_yolocloud.msg import ObjectInfoArray


class ObjectTracker(object):
    def __init__(self, wait=0.0):

        self.infoarray = ObjectInfoArray()
        self.image_pub = rospy.Publisher("objet_tracker/particle_filter",Image,queue_size=10)
        self.objimage_pub1 = rospy.Publisher("object1_photo",Image,queue_size=10)
        self.objimage_pub2 = rospy.Publisher("object2_photo",Image,queue_size=10)
        self.objimage_pub3 = rospy.Publisher("object3_photo",Image,queue_size=10)
        # rospy.Service('/relearn_clothes', learn_clothes, self.learning_dataset)
        dirname = os.path.dirname(__file__)
        self.known_folder = dirname
        self.savepicture=False
        # self.filename1=self.known_folder+"/object2.png"
        self.filename1=self.known_folder+"/clothes0.png"
        template = cv2.imread(self.filename1) #Load the image
        self.my_mask_analyser = BinaryMaskAnalyser()
        self.my_back_detector = BackProjectionColorDetector()
        self.my_back_detector.setTemplate(template) #Set the template 
        tot_particles =2000
        self.std=20;
        self.my_particle = ParticleFilter(480, 640, tot_particles)
        # self.my_particle = ParticleFilter(640, 480, tot_particles)
        self.noise_probability = 0.10 #in range [0, 1.0]
        self.save_time = rospy.get_time()

        image_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        # image_topic = "/camera/rgb/image_rect_color"
	rospy.Subscriber(image_topic, Image, self.image_callback)
        objinfo_topic = "/objects_info"
	rospy.Subscriber(objinfo_topic, ObjectInfoArray, self.object_callback)
        self.filter_result_pub=rospy.Publisher('/object_tracker/positions',Float32MultiArray,queue_size=10)
        self.bridge = CvBridge()

    def learning_dataset(self,req):
        template = cv2.imread(self.filename1) #Load the image
        self.my_mask_analyser = BinaryMaskAnalyser()
        self.my_back_detector = BackProjectionColorDetector()
        self.my_back_detector.setTemplate(template) #Set the template 
        tot_particles =2500
        self.std=20;
        self.my_particle = ParticleFilter(480, 640, tot_particles)
        self.noise_probability = 0.10 #in range [0, 1.0]
        print("training done")
        return learn_clothesResponse()

    def object_callback(self,msg):
        # rospy.loginfo("object_callback")
        self.infoarray=msg

    def crop_objects(self, image_data):
        image_batch = image_data
        objfiles = []
        for index in range(len(self.infoarray.objectinfos)):
            cut_x=int(self.infoarray.objectinfos[index].bbox.x)
            cut_y=int(self.infoarray.objectinfos[index].bbox.y)
            crop_width=int(self.infoarray.objectinfos[index].bbox.width)
            crop_height=int(self.infoarray.objectinfos[index].bbox.height)
            object_crop= image_batch[cut_y:cut_y+crop_height, cut_x:cut_x+crop_width]
            objfiles.append(object_crop)

        # if self.savepicture==False:
           # self.save_pictures(objfiles)

        #photo update every 15 seconds
        cur_time = rospy.get_time()
        duration = cur_time - self.save_time
        if duration >10.0:
            if self.savepicture==False:
                self.save_pictures(objfiles)
            else:
                return objfiles
            
        return objfiles
    def save_pictures(self, images):

        for index in range(len(images)):
            self.savedname = self.known_folder+"/object" + str(index) + ".png"
            cv2.imwrite(self.savedname, images[index])
            print "file_path", self.savedname

        
        if len(images)>0:
            print('save picture')
            self.savepicture=True
            #save Time
            self.save_time=rospy.get_time()

            template = cv2.imread(self.savedname) #Load the image
            self.my_mask_analyser = BinaryMaskAnalyser()
            self.my_back_detector = BackProjectionColorDetector()
            self.my_back_detector.setTemplate(template) #Set the template 
            tot_particles =2000
            self.std=20;
            self.my_particle = ParticleFilter(480, 640, tot_particles)
            self.noise_probability = 0.10 #in range [0, 1.0]
            print("training done")
            rospy.sleep(1)


	
    def image_callback(self,msg):
        # print('image_callback: ')
        
        try:
            frame   = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            objectsphotos= self.crop_objects(frame)

            if len(objectsphotos)>0:
                self.objimage_pub1.publish(self.bridge.cv2_to_imgmsg(objectsphotos[0], "bgr8"))
            
            if len(objectsphotos)>1:
                self.objimage_pub2.publish(self.bridge.cv2_to_imgmsg(objectsphotos[1], "bgr8"))

            if len(objectsphotos)>3:
                self.objimage_pub3.publish(self.bridge.cv2_to_imgmsg(objectsphotos[3], "bgr8"))

            frame_mask = self.my_back_detector.returnMask(frame, morph_opening=True, blur=True, kernel_size=5, iterations=2)
        
            if(self.my_mask_analyser.returnNumberOfContours(frame_mask) > 0):
                x_rect,y_rect,w_rect,h_rect = self.my_mask_analyser.returnMaxAreaRectangle(frame_mask)
                x_center, y_center = self.my_mask_analyser.returnMaxAreaCenter(frame_mask)
                #Adding noise to the coords
                coin = np.random.uniform()
                if(coin >= 1.0-self.noise_probability): 
                    x_noise = int(np.random.uniform(-300, 300))
                    y_noise = int(np.random.uniform(-300, 300))
                else: 
                    x_noise = 0
                    y_noise = 0
                x_rect += x_noise
                y_rect += y_noise
                x_center += x_noise
                y_center += y_noise
                cv2.rectangle(frame, (x_rect,y_rect), (x_rect+w_rect,y_rect+h_rect), [255,0,0], 2) #BLUE rect

                #Predict the position of the target
                self.my_particle.predict(x_velocity=0, y_velocity=0, std=self.std)

                #Drawing the particles.
                self.my_particle.drawParticles(frame)

               #Estimate the next position using the internal model
                x_estimated, y_estimated, _, _ = self.my_particle.estimate()
                cv2.circle(frame, (x_estimated, y_estimated), 3, [0,255,0], 5) #GREEN dot

                #publish_filter result
                filter_msg=Float32MultiArray()
                filter_msg.data.append(x_estimated)
                filter_msg.data.append(y_estimated)
                self.filter_result_pub.publish(filter_msg)

               #Update the filter with the last measurements
                self.my_particle.update(x_center, y_center)

               #Resample the particles
                self.my_particle.resample()

               #Writing in the output file
               # out.write(frame)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))


            # cv2.imshow("image_window", frame)
            # cv2.imshow('Mask', frame_mask) #show on window
            # cv2.waitKey(3)
            # if cv2.waitKey(3) & 0xFF == ord('q'):
                # break #Exit when Q is pressed

        except CvBridgeError, e:
            print(e)


    def listener(self,wait=0.0):
        rospy.spin() 


if __name__ == '__main__':
        rospy.init_node('particle_filter_test')
	# print("Initialize node")
        tracker_manager = ObjectTracker(sys.argv[1] if len(sys.argv) >1 else 0.0)
	tracker_manager.listener()	
