#!/usr/bin/env python

import rospy
import math
import sys
import numpy as np
from numpy.random import uniform

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

#Create particles
width =10
height=10
depth =10
x_velocity=0.02
x_velocity=0.04
std=0.01
N=300
particles = np.empty((N,2))
particles[:,0]=uniform(-width,width, size=N) #init x coord
particles[:,1]=uniform(-height,height, size=N) #init y coord
cloud_sets=[]
header = std_msgs.msg.Header()
# header.stamp = rospy.Time.now()
# header.frame_id = 'map'
scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_sets)


    #

def predict(x_veloity, y_velocity, std):
    particles[:, 0] += x_velocity + (np.random.randn(len(particles)) * std) #predict the X coord
    particles[:, 1] += y_velocity + (np.random.randn(len(particles)) * std) #predict the Y coord
    # print "predict"

def update(x,y):
    position = np.empty((len(particles), 2))
    position[:, 0].fill(x)
    position[:, 1].fill(y)
    #1- We can take the difference between each particle new
    #position and the measurement. In this case is the Euclidean Distance.
    distance = np.linalg.norm(particles - position, axis=1)
    #2- Particles which are closer to the real position have smaller
    #Euclidean Distance, here we subtract the maximum distance in order
    #to get the opposite (particles close to the real position have
    #an higher wieght)
    max_distance = np.amax(distance)
    distance = np.add(-distance, max_distance)
    #3-Particles that best predict the measurement 
    #end up with the highest weight.
    weights.fill(1.0) #reset the weight array
    weights *= distance
    #4- after the multiplication the sum of the weights won't be 1. 
    #Renormalize by dividing all the weights by the sum of all the weights.
    weights += 1.e-300 #avoid zeros
    weights /= sum(self.weights) #normalize

def estimate():
    x_mean = np.average(particles[:, 0], weights=weights, axis=0).astype(float)
    y_mean = np.average(particles[:, 1], weights=weights, axis=0).astype(float)
    return x_mean, y_mean, 0, 0

def updatepcl():
    cloud_sets=[]
    for x_particle, y_particle in particles.astype(float):
        y_noise=np.random.uniform(0,1)
        x_noise=np.random.uniform(0,1)
        cloud_sets.append([x_particle+x_noise,y_particle+y_noise, 0.5])

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_sets)
    #


if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''
    rospy.init_node('pcl2_pub_example')
    pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2)
    rospy.loginfo("Initializing sample pcl2 publisher node...")
    #give time to roscore to make the connections
    rospy.sleep(1.)
    cloud_points = [[1.0, 1.0, 0.0],[1.0, 2.0, 0.0]]

    #header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = '/map'
    # particles[:,2]=uniform(0,depth, size=N) #init z coord
    weights = np.array([1.0/N]*N)
    #create pcl from points

    cloud_sets=[]
    for x_particle, y_particle in particles.astype(float):
        cloud_sets.append([x_particle,y_particle, 0.5])

    scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_sets)
    #publish    
    # rospy.loginfo("happily publishing sample pointcloud.. !")
    count=0
    while not rospy.is_shutdown():

        cloud_sets=[]
        for x_particle, y_particle in particles.astype(float):
            y_noise=np.random.uniform(0,1)
            x_noise=np.random.uniform(0,1)
            cloud_sets.append([x_particle+x_noise,y_particle+y_noise, 0.5])

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_sets)
        # updatepcl()
        pcl_pub.publish(scaled_polygon_pcl)
        rospy.loginfo("happily publishing sample pointcloud.. !")
        # print "publish"
        # count
