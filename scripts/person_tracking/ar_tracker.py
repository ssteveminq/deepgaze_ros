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

import cv2
import roslib
import sys
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import select, termios, tty
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from people_msgs.msg import PositionMeasurementArray
from std_msgs.msg import String
import std_msgs.msg
from os import listdir
import os
# import tensorflow as tf

# from deepgaze.color_detection import BackProjectionColorDetector
# from deepgaze.mask_analysis import BinaryMaskAnalyser
from deepgaze.object3d_tracking import ParticleFilter
# from deepgaze.motion_tracking import ParticleFilter

class ArTracker(object):
    def __init__(self, wait=0.0):

        # template = cv2.imread('orange.png') #Load the image
        tot_particles =300
        self.std=0.05;
        self.my_particle = ParticleFilter(20, 20, tot_particles)
        self.noise_probability = 0.10 #in range [0, 1.0]

        posearray_topic="/ar_tracker_measurements"
	rospy.Subscriber(posearray_topic, PositionMeasurementArray, self.PositionMeasurementCb)

        self.pcl_pub=rospy.Publisher("/particle_samples",PointCloud2,queue_size=50)

        # self.bridge = CvBridge()
    def PositionMeasurementCb(self,msg):
        #recieve poses array from measurement
        poses_array=msg.people
        detected_people=len(poses_array)
        if(detected_people>0):
            print "person detected"
        else:
            return

        x_center=poses_array[0].pos.x
        y_center=poses_array[0].pos.y
        # z_center=poses_array[0].position.z
        #data association

        #Let's say we are tracking som ar tags tracking chairs
        #update center of 3d objects center

        #add noises
        coin = np.random.uniform()
        if(coin >= 1.0-self.noise_probability): 
            x_noise = float(np.random.uniform(-0.05, 0.05))
            y_noise = float(np.random.uniform(-0.05, 0.05))
                # z_noise = int(np.random.uniform(-300, 300))
        else: 
            x_noise = 0
            y_noise = 0
                # z_noise = 0
        x_center += x_noise
        y_center += y_noise
            # z_center += z_noise

        #Predict the position of the target
        self.my_particle.predict(x_velocity=0, y_velocity=0, std=self.std)

        #Drawing the particles.
        self.visualizeparticles()
        # self.my_particle.drawParticles(frame)

        #Estimate the next position using the internal model
        x_estimated, y_estimated, _, _ = self.my_particle.estimate()
        # cv2.circle(frame, (x_estimated, y_estimated), 3, [0,255,0], 5) #GREEN dot

        #Update the filter with the last measurements
        self.my_particle.update(x_center, y_center)

        #Resample the particles
        self.my_particle.resample()



    def visualizeparticles(self):
        cloud_sets=[]
        for x_particle, y_particle in self.my_particle.particles.astype(float):
            cloud_sets.append([x_particle,y_particle,0.5])
        
        header=std_msgs.msg.Header()
        header.stamp=rospy.Time.now()
        header.frame_id='map'
        particle_pcl=pcl2.create_cloud_xyz32(header,cloud_sets)
        self.pcl_pub.publish(particle_pcl)
            
	

    def listener(self,wait=0.0):
        rospy.spin() 


if __name__ == '__main__':
        rospy.init_node('Ar_particle_filter_test')
	# print("Initialize node")
        tracker_manager = ArTracker(sys.argv[1] if len(sys.argv) >1 else 0.0)
	tracker_manager.listener()	

