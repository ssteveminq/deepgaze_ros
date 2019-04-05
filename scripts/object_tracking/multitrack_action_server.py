#!/usr/bin/env python

# ROS version written by Minkyu Kim

import cv2
import roslib
import sys
import rospy
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
import select, termios, tty
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import JointState
import sensor_msgs.point_cloud2 as pcl2
from os import listdir
import os
from os.path import isfile, join, exists
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from deepgaze_ros.msg import ModeConverterAction, ModeConverterResult

import actionlib
from deepgaze_ros.msg import *
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray, Int8MultiArray
from nav_msgs.msg import OccupancyGrid
from numpy.random import uniform
from deepgaze.color_detection import BackProjectionColorDetector
from deepgaze.mask_analysis import BinaryMaskAnalyser
from deepgaze.motion_tracking import ParticleFilter
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion, PointStamped
from pointcloud_processing_msgs.msg import ObjectInfo, ObjectInfoArray
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from deepgaze.object3d_tracking import ParticleFilter3D
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from deepgaze.geometry_tools import projection_point_to_plane 
from deepgaze.geometry_tools import tangent_point_circle_extpoint 

_MAP_TF='map'
_SENSOR_TF ='head_rgbd_sensor_rgb_frame'
OCC_RADIUS=0.2
MEAS_SAMPLENUM=50

class ObjectTracker(object):
    def __init__(self, name,  wait=0.0):

        self._action_name= name
        #ROS action started
        self._as = actionlib.SimpleActionServer(self._action_name, deepgaze_ros.msg.MultiTrackAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        self.sm_result = ModeConverterResult()
        self.sm_feedback= ModeConverterFeedback()
        self.sm_feedback.opt_pose= PoseStamped()
        self._action_name_sm= 'state_machine'
        self.sm_as = actionlib.SimpleActionServer(self._action_name_sm, deepgaze_ros.msg.ModeConverterAction, execute_cb=self.sm_execute_cb, auto_start = False)
        self.sm_as.start()
        rospy.loginfo("sm started")


        self.smlasttime=rospy.get_time()
        self.last_navitime=rospy.get_time()
        self.last_imagetime=rospy.get_time()
        
        self.last_context_mode=0
        self.search_mode=0
        self.map_received=False
        self.navi_mode=0
        """
        ---context_mode index---
        0: tracking mode
        1: searching mode(missing) 
        2: occlusion from identifiable object
        3: occlusiong from non-ideintifiable object
        """
        self.context_modes=[]
        self.occ_point=Point()

        self.last_observations=[]
        self.last_targetlocations=[]
        self.last_targetlocations.append(Point(0.0,0.0,0.0))
        # self.last_targetlocations.append(Point(0.0,0.0,0.0))

        self.context_fov_locations=[]
        self.last_callbacktime=rospy.get_time()
        self.context_activate=0

        self.outoffovpoint= Point() 
        self.outoffovpoint.x= 1.5 
        self.outoffovpoint.y= -1.5 
        self.outoffovpoint2= Point() 
        self.outoffovpoint2.x= 1.5 
        self.outoffovpoint2.y= 1.5 

        self.robot_pose=np.zeros((5,1))
        self.Init_track=False
	self.target_distance=0.5
        # self.target_ids=[]
        self.target_strings=[]
        self.trackers=[]
        self.trackers3d=[]
        self.templete_trackers=[]
        self.my_back_detectors=[]
        self.mask_analysers=[]
        # self.frame=[]
        self.color_track=False
        # self.color_track=True
        self.infoarray = ObjectInfoArray()
        self.image_pub = rospy.Publisher("objet_tracker/particle_filter",Image,queue_size=10)
        self.objimage_pub1 = rospy.Publisher("object1_photo",Image,queue_size=10)
        self.objimage_pub2 = rospy.Publisher("object2_photo",Image,queue_size=10)
        self.objimage_pub3 = rospy.Publisher("object3_photo",Image,queue_size=10)

        self.fov_points_pub=rospy.Publisher("/out_fov_points",PointStamped,queue_size=30)
        self.tangent_point1_pub=rospy.Publisher("/tanget_points1",PointStamped,queue_size=30)
        self.tangent_point2_pub=rospy.Publisher("/tanget_points2",PointStamped,queue_size=30)
        self.estimated_point_pub=rospy.Publisher("/estimated_target",PoseStamped,queue_size=30)
        self.sample_meas_pub=rospy.Publisher("/measurements_sample",PoseArray,queue_size=30)
        self.avg_point_pub=rospy.Publisher("/avg_point",PoseStamped,queue_size=30)
        self.opt_pose_pub=rospy.Publisher("/opt_pose",PoseStamped,queue_size=30)
        self.head_command_pub=rospy.Publisher('desired_head_pan',Float32,queue_size=10)
        self.pcl_pub=rospy.Publisher("/particle_samples",PointCloud2,queue_size=50)
        self.pcl_pub2=rospy.Publisher("/particle_samples2",PointCloud2,queue_size=50)
        self.occpcl_pub=rospy.Publisher("/occ_particles",PointCloud2,queue_size=50)

        self.last_occfovtime = rospy.get_time()
        # rospy.Service('/relearn_clothes', learn_clothes, self.learning_dataset)
        dirname = os.path.dirname(__file__)
        # self.knooutoffovpoint2.x= 1.5 

        self.robot_pose=np.zeros((5,1))
        self.Init_track=False
        # self.target_ids=[]
        self.target_strings=[]
        self.trackers=[]
        self.trackers3d=[]
        self.templete_trackers=[]
        self.my_back_detectors=[]
        self.mask_analysers=[]
        # self.frame=[]
        self.color_track=False
        # self.color_track=True
        # self.infoarray = ObjectInfoArray()
        # self.image_pub = rospy.Publisher("objet_tracker/particle_filter",Image,queue_size=10)
        # self.objimage_pub1 = rospy.Publisher("object1_photo",Image,queue_size=10)
        # self.objimage_pub2 = rospy.Publisher("object2_photo",Image,queue_size=10)
        # self.objimage_pub3 = rospy.Publisher("object3_photo",Image,queue_size=10)

        # self.estimated_point_pub=rospy.Publisher("/estimated_target",PoseStamped,queue_size=30)
        # self.sample_meas_pub=rospy.Publisher("/measurements_sample",PoseArray,queue_size=30)
        # self.avg_point_pub=rospy.Publisher("/avg_point",PoseStamped,queue_size=30)
        # self.opt_pose_pub=rospy.Publisher("/opt_pose",PoseStamped,queue_size=30)
        # self.head_command_pub=rospy.Publisher('desired_head_pan',Float32,queue_size=10)
        # self.pcl_pub=rospy.Publisher("/particle_samples",PointCloud2,queue_size=50)

        dirname = os.path.dirname(__file__)
        self.known_folder = dirname
        self.savepicture=False
        # self.filename1=self.known_folder+"/object2.png"
        self.filename1=self.known_folder+"/clothes0.png"
        template = cv2.imread(self.filename1) #Load the image

        #variables for particlefilters
        self.tot_particles =2000
        self.tot_particles3d =500
        self.std=20;
        self.std3d=0.12;
        self.noise_probability = 0.10 #in range [0, 1.0]
        self.save_time = rospy.get_time()
        self.s1_particles=[]
        self.s2_particles=[]
        self.s1_weights=[]
        self.s2_weights=[]
        self.target_z=0.9
        self.avg_x=0
        self.avg_y=0

        self.dynamic_map=OccupancyGrid()
        self.static_map=OccupancyGrid()
        self.meas_sample_num=MEAS_SAMPLENUM
        self.meas_samples=np.empty((self.meas_sample_num,3))
        self.occ_raius=0.1

        self.bridge = CvBridge()
        # image_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        image_topic = "/hsrb/head_rgbd_sensor/rgb/image_raw"
        # image_topic = "/camera/rgb/image_color"
        # image_topic = "/camera/rgb/image_raw"
	rospy.Subscriber(image_topic, Image, self.image_callback)
        handimage_topic = "/hsrb/hand_camera/image_raw"
	# rospy.Subscriber(handimage_topic, Image, self.handimage_callback)
        objinfo_topic = "/ObjectInfos"
	rospy.Subscriber(objinfo_topic, ObjectInfoArray, self.object_callback)
        darknet_topic = "/darknet_ros/bounding_boxes"
	rospy.Subscriber(darknet_topic, BoundingBoxes, self.yolo_callback)
        darknet_topic_hand = "/darknet_ros_hand/bounding_boxes"
	rospy.Subscriber(darknet_topic_hand, BoundingBoxes, self.yolo_hand_callback)

        robot_pose_topic='global_pose'
        rospy.Subscriber(robot_pose_topic, PoseStamped, self.robot_pose_Cb)
        jointstates_topic='hsrb/joint_states'
	rospy.Subscriber(jointstates_topic, JointState, self.joint_state_Cb)
        map_topic="/projected_map"
        rospy.Subscriber(map_topic, OccupancyGrid, self.map_Cb)
        bottle_topic="/bottle_center"
        # rospy.Subscriber(bottle_topic, PointStamped, self.bottle_Cb)

        staticmap_topic="/static_obstacle_map_ref"
        rospy.Subscriber(staticmap_topic, OccupancyGrid, self.staticmap_Cb)
 
        self.navi_cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        # self.navi_cli.wait_for_server()

    def sm_execute_cb(self, goal):
        curtime = rospy.get_time()
        self.sm_result.current_mode=-1

        if self.Init_track:
            if (curtime - self.smlasttime)>2.5:
                self.sm_result.current_mode=self.context_modes[0]
                self.smlasttime=rospy.get_time()
                if self.sm_result.current_mode>1:
                    self.sm_as.publish_feedback(self.sm_feedback)
                    #TODO: put feedback_Info
            else:
                self.sm_result.current_mode=-1

            self.sm_as.publish_feedback(self.sm_feedback)
            self.sm_as.set_succeeded(self.sm_result)
        else:
            self.sm_as.publish_feedback(self.sm_feedback)
            self.sm_as.set_succeeded(self.sm_result)


    def execute_cb(self, goal):
        idcount=0
        for name in goal.target_labels:
            rospy.loginfo("target set : id :%s",  name)
            self.target_strings.append(name)
            for index in range(len(self.infoarray.objectinfos)):
                if self.infoarray.objectinfos[index].label ==name:
                    rospy.loginfo("tracking object label : %s",name)
                    last_targetlocation = self.infoarray.objectinfos[index].center
                    self.last_targetlocations[idcount]=last_targetlocation
                    idcount+=1

            # for index in range(len(self.infoarray.objectinfos)):
                # if self.infoarray.objectinfos[index].id == id:
                    # self.target_strings.append(self.infoarray.objectinfos[index].label)
                    # rospy.loginfo("tracking object label : %s",self.infoarray.objectinfos[index].label)
                    # last_targetlocation = self.infoarray.objectinfos[index].center
                    # self.last_targetlocations.append(last_targetlocation)

            #2D tracking
            indiv_particle=ParticleFilter(480, 640, self.tot_particles)
            self.trackers.append(indiv_particle)

            #3D tracking
            indiv_particle3d=ParticleFilter3D(10, 10, self.tot_particles3d)
            self.trackers3d.append(indiv_particle3d)
            #TODO: how we can define context? multiple context? 
            # self.context_particle = ParticleFilter3D(10,10,tot_particles)
            self.last_observations.append(rospy.get_time()) #update last obsservation time for each object
            self.context_modes.append(0) #0: tracking mode 1: searching mode(missing) , 2:occlusion from identifiable object 3: occlusiong from non-ideintifiable object
 

        rospy.loginfo("tracking initiated!!")
        self.Init_track=True

        #set initial position of targets
        # for id in self.target_ids:
            # for index in range(len(self.infoarray.objectinfos)):
                # if self.infoarray.objectinfos[index].id == id:

                    # cut_x=int(self.infoarray.objectinfos[index].bbox.x)
                    # cut_y=int(self.infoarray.objectinfos[index].bbox.y)
                    # crop_width=int(self.infoarray.objectinfos[index].bbox.width)
                    # crop_height=int(self.infoarray.objectinfos[index].bbox.height)
                    # object_crop= image_batch[cut_y:cut_y+crop_height, cut_x:cut_x+crop_width]
                    # objfiles.append(object_crop)

        self._as.set_succeeded()

    def staticmap_Cb(self,msg):
        self.static_map = msg
        self.map_received=True
        
    def map_Cb(self,msg):
        self.dynamic_map = msg

    def navigation_action(self,goal_x,goal_y,goal_yaw):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(goal_x, goal_y, 0)
        quat = quaternion_from_euler(0, 0, goal_yaw)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        # send message to the action server
        self.navi_cli.send_goal(goal)

        # wait for the action server to complete the order
        self.navi_cli.wait_for_result()

        # print result of navigation
        result_action_state = self.navi_cli.get_state()
        self.last_navitime=rospy.get_time()

        return result_action_state 


    def robot_pose_Cb(self, msg):
        #self_robot_pose[0] = robot_position_x
        #self_robot_pose[1] = robot_position_y
        #self_robot_pose[2] = robot_theta_yaw
        #self_robot_pose[3] = head_pan_angle
        #self_robot_pose[4] = head_tilt_angle

        self.robot_pose[0]=msg.pose.position.x
        self.robot_pose[1]=msg.pose.position.y
        robot_orientation=msg.pose.orientation

        #get yaw angle from quarternion
        orientation_list=[robot_orientation.x, robot_orientation.y, robot_orientation.z,robot_orientation.w]
        roll,pitch,yaw=euler_from_quaternion(orientation_list)
        self.robot_pose[2]=yaw
        # rospy.loginfo("global theta: %.3lf", self.robot_pose[2])

    def joint_state_Cb(self, msg):
        #self_robot_pose[3] = head_pan_angle
        #self_robot_pose[4] = head_tilt_angle
        self.robot_pose[3]=msg.position[9]
        self.robot_pose[4]=msg.position[10]
        # rospy.loginfo("head angle: %.3lf", self.robot_pose[3])


    def learning_dataset(self,req):
        template = cv2.imread(self.filename1) #Load the image
        self.my_mask_analyser = BinaryMaskAnalyser()
        self.my_back_detector = BackProjectionColorDetector()
        self.my_back_detector.setTemplate(template) #Set the template 
        tot_particles =2000
        self.std=20;
        self.my_particle = ParticleFilter(480, 640, tot_particles)
        self.noise_probability = 0.10 #in range [0, 1.0]
        print("training done")
        return learn_clothesResponse()

    def yolo_callback(self,msg):
        self.bounding_boxes = msg.bounding_boxes
        '''
        if self.Init_track:
            for bbox in self.bounding_boxes:
                for id in range(len(self.target_strings)):
                    # if bbox.Class == self.target_strings[id]:

                    desired_head_pan= self.robot_pose[3]
                    if bbox.Class == 'bottle':
                        self.update_trackers(0,bbox.xmin,bbox.xmax,bbox.ymin,bbox.ymax)
                        box_center = Point((bbox.xmin+bbox.xmax)/2, (bbox.xmin+bbox.xmax)/2, self.target_z)
                        if box_center.x<200:
                            ee_desired_cmd=1
                            desired_head_pan = self.robot_pose[3]+0.2
                            rospy.loginfo("deisred_head : left")
                        elif box_center.x<100:
                            desired_head_pan = self.robot_pose[3]+0.5
                            rospy.loginfo("deisred_head : left")
                        elif box_center.x>450:
                            ee_desired_cmd=2
                            desired_head_pan = self.robot_pose[3]-0.25
                            rospy.loginfo("deisred_head : right")
                        elif box_center.x>560:
                            ee_desired_cmd=2
                            desired_head_pan = self.robot_pose[3]-0.5
                            rospy.loginfo("deisred_head : right")
                        else:
                            ee_desired_cmd=0
                            desired_head_pan= self.robot_pose[3]
                            
                        #publish the desired joint angles
                        desired_pan_joint = Float32()
                        desired_pan_joint.data= desired_head_pan
                        self.head_command_pub.publish(desired_pan_joint)
        '''



    def yolo_hand_callback(self,msg):
        self.bounding_boxes_hand = msg.bounding_boxes


        #set target_observations to False
        # target_observations=[]
        # for id in range(len(self.target_ids)):
            # target_observations.append(False)
            
        # if self.Init_track:
            # for bbox in self.bounding_boxes:
                # for id in range(len(self.target_strings)):
                    # if bbox.Class == self.target_strings[id]:
                        # self.update_trackers(id,bbox.xmin,bbox.xmax,bbox.ymin,bbox.ymax)
                        # self.last_observations[id]=rospy.get_time()
                        ## self.trackers[id].resample()

        #check if there didn't come obsrvations

    # def Update_NoMeasurement_CurrentConfig(self,idx):


    # def Update_NOMeasurement_Filter3D(self,cur_fov):



    def Update_Measurement_Filter3D(self,idx, x_center, y_center):
        #Update the filter with the last measurements
        
        #add noises to measurements
        coin = np.random.uniform()
        if(coin >= 1.0-self.noise_probability): 
            x_noise = float(np.random.uniform(-0.15, 0.15))
            y_noise = float(np.random.uniform(-0.15, 0.15))
                # z_noise = int(np.random.uniform(-300, 300))
        else: 
            x_noise = 0
            y_noise = 0
        x_center += x_noise
        y_center += y_noise

        self.trackers3d[idx].update(x_center, y_center)
        # print self.trackers3d[idx.weights
        current_entropy=self.trackers3d[idx].get_entropy()
        weight_var=np.var(100*self.trackers3d[idx].weights, dtype=np.float64)
        # rospy.loginfo("variance of particles before: %.6lf, entropy: %.3lf",weight_var,current_entropy)
        try:
            self.trackers3d[idx].resample('residual')
        except:
            rospy.loginfo("resample failed")
        # self.trackers3d[idx.resample('residual')
        weight_var=np.var(100*self.trackers3d[idx].weights, dtype=np.float64)
        # current_entropy=self.trackers3d[idx].get_entropy()
        # weight_var=np.var(self.my_particle.weights)
        # rospy.loginfo("variance of particles after: %.6lf, entropy: %.3lf",weight_var,current_entropy)

    def bottle_Cb(self,msg):

        if self.Init_track:
            if len(msg.poses)>0:
                self.last_targetlocations[0]=msg.point
                self.last_observations[0]=rospy.get_time()
                self.Estimate_Filter3D()
                self.Update_Measurement_Filter3D(0, msg.point.x,
                    msg.point.y)

    def object_callback(self,msg):
        self.infoarray=msg
        if self.Init_track:
            #check if received objectinfo is target or not
            for id in range(len(self.target_strings)):
                for index in range(len(self.infoarray.objectinfos)):
                    if self.infoarray.objectinfos[index].label == self.target_strings[id]:
                        # rospy.loginfo("object info received: %s ", self.target_strings[id])
                        self.last_targetlocations[id]=self.infoarray.objectinfos[index].center
                        self.sm_feedback.last_target_position = self.last_targetlocations[id]
                        
			self.target_distance = self.infoarray.objectinfos[index].average_depth
                        #updating bounding box 
                        # if self.infoarray.objectinfos[index].no_observation==False:
                            # self.last_observations[id]=False
                        last_time = self.infoarray.objectinfos[index].last_time
                        # rospy.loginfo("target %s, last time : %.3lf", self.infoarray.objectinfos[index].label, last_time)
                        self.last_observations[id]=last_time.secs
                        cur_time = rospy.get_time()
                        
                        #case: if measurement comes within 1.0 sec
                        if (cur_time - last_time.secs)<1.0:
                            self.context_modes[id]=0
                            self.Estimate_Filter3D_idx(id)
                            self.Update_Measurement_Filter3D(id, self.infoarray.objectinfos[index].center.x,
                                                         self.infoarray.objectinfos[index].center.y)
                            try:
                                self.trackers[id].resample()
                            except:
                                rospy.loginfo("resample failed")

                        #case: if measurement doesn't come within 1.0 sec
                        else:
                            # if the target is missing because of occlusion
                            if self.infoarray.objectinfos[index].occ_objects==True or self.infoarray.objectinfos[index].depth_change ==True:
                                # self.context_modes[id]=2
                                occ_reason = self.infoarray.objectinfos[index].occ_label
                                # rospy.loginfo("object are occluded by %s, converting to searching_mode", occ_reason)
                                for occ_index in range(len(self.infoarray.objectinfos)):
                                    #occulusion because of identifiable object
                                    if self.infoarray.objectinfos[occ_index].label == occ_reason:
                                       self.context_modes[id]=2
                                       self.occ_point.x = self.infoarray.objectinfos[occ_index].center.x
                                       self.occ_point.y = self.infoarray.objectinfos[occ_index].center.y
                                       self.occ_point.z = self.infoarray.objectinfos[occ_index].center.z
                                       # rospy.loginfo("estimated from known object %s", occ_reason)
                                       self.Estimate_Filter3D_point(id,self.occ_point)
                                    else:
                                        #occulusion because of non-identifiable object: use last observations
                                       self.context_modes[id]=3
                                       self.occ_point = self.last_targetlocations[id]
                                       # self.occ_point.x= self.occ_point.x+0.12
                                       self.Estimate_Filter3D_point(id,self.last_targetlocations[index])
                                       # rospy.loginfo("estimated from uknown -from last observation")
                            else:
                            # if the target is missing because of it's fast speed
                                if (cur_time - last_time.secs)>5.0:
                                    self.context_modes[id]=1
                                    rospy.loginfo("----missing case---")
                                    outputpointset=[]
                                    # update outoffovpoint 
                                    self.get_fov_outpoints()
                                    # outputpointset.append(self.outoffovpoint)
                                    # outputpointset.append(self.outoffovpoint2)
                                    self.Estimate_Filter3D_pointset(id,self.get_fov_outpoints())
                                else:
                                    self.context_modes[id]=0
                                    #TODO: Check this case 
                            # if the target is missing because of its high-speed
                            # self.infoarray.objectinfos[occ_index].center.y)

            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.rvizframe, "bgr8"))
    def get_fov_outpoints(self):
        #from current configurations get field of view : exploring like frontier-based navigation
        #getNearFovRegion()
        FOV_horizontal=58.0*2*math.pi/360.0
        FOV_vertical=45.0*2*math.pi/360.0

        plane_height = self.target_distance*2*math.tan(0.5*FOV_vertical)
        plane_width = self.target_distance*2*math.tan(0.5*FOV_horizontal)

        #Defined in sensor frame
        left_point=Point(-0.85*plane_width,0.0, 1.2*self.target_distance)
        right_point=Point(0.85*plane_width,0.0, 1.2*self.target_distance)

        listener = tf.TransformListener()
        listener.waitForTransform(_SENSOR_TF, _MAP_TF, rospy.Time(0),rospy.Duration(4.0))
        head_point=PointStamped()
        head_point.header.frame_id = _SENSOR_TF
        head_point.header.stamp =rospy.Time(0)
        head_point.point=left_point
        output_leftpoint=listener.transformPoint(_MAP_TF,head_point)

        rhead_point=PointStamped()
        rhead_point.header.frame_id = _SENSOR_TF
        rhead_point.header.stamp =rospy.Time(0)
        rhead_point.point=right_point
        output_rightpoint=listener.transformPoint(_MAP_TF,rhead_point)

        outpointset=[]
        outpointset.append(output_leftpoint.point)
        outpointset.append(output_rightpoint.point)
        self.fov_points_pub.publish(output_leftpoint)
        return outpointset


    def crop_objects(self, image_data):
        image_batch = image_data
        objfiles = []
        for index in range(len(self.infoarray.objectinfos)):
            for id in range(len(self.target_self.ids)):
                for index in range(len(self.infoarray.objectinfos)):
                    if self.infoarray.objectinfos[index].label == self.target_strings[id]:
                        cut_x=int(self.infoarray.objectinfos[index].bbox.x)
                        cut_y=int(self.infoarray.objectinfos[index].bbox.y)
                        crop_width=int(self.infoarray.objectinfos[index].bbox.width)
                        crop_height=int(self.infoarray.objectinfos[index].bbox.height)
                        object_crop= image_batch[cut_y:cut_y+crop_height, cut_x:cut_x+crop_width]
                        objfiles.append(object_crop)

        if self.savepicture==False:
           self.save_pictures(objfiles)
           self.save_picture=True

        #photo update every 15 seconds
        cur_time = rospy.get_time()
        duration = cur_time - self.save_time
        #not required for now ---- reset templete
        # if duration >20.0:
            # if self.savepicture==False:
                # self.save_pictures(objfiles)
            # else:
                # return objfiles
            
        return objfiles
    def save_pictures(self, images):
        # self.my_back_detectors=[]
        # for index in range(len(images)):
        for id in range(len(self.target_strings)):
            self.savedname = self.known_folder+"/object" + str(id) + ".png"
            cv2.imwrite(self.savedname, images[id])
            print "file_path", self.savedname

            if len(images)>0:
                print('save picture')
                self.savepicture=True
            #save Time
            # rospy.sleep(1.5)
            template = cv2.imread(self.savedname) #Load the image
            my_mask_analyser = BinaryMaskAnalyser()
            self.mask_analysers.append(my_mask_analyser)
            my_back_detector = BackProjectionColorDetector()
            my_back_detector.setTemplate(template) #Set the template 
            self.my_back_detectors.append(my_back_detector)
            tot_particles =1500
            self.std=20;
            hog_particle = ParticleFilter(480, 640, tot_particles)
            self.templete_trackers.append(hog_particle)
            self.noise_probability = 0.10 #in range [0, 1.0]
            # rospy.sleep(1)

        # print("training done")
        self.save_time=rospy.get_time()

    def handimage_callback(self,msg):
        # print('image_callback: ')
        try:
            self.handframe   = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            # self.rvizframe   = self.frame
            # print('image_callback: ')
            if self.Init_track:
                for bbox in self.bounding_boxes_hand:
                    for id in range(len(self.target_strings)):
                        if bbox.Class == 'cup':
                            self.update_trackers_handimage(1,bbox.xmin,bbox.xmax,bbox.ymin,bbox.ymax)


            cv2.namedWindow("handimage_window", cv2.WINDOW_NORMAL)
            # cv2.moveWindow(winname, 50,40)
            cv2.imshow("handimage_window", self.handframe)
            cv2.waitKey(1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                exit()
               # break #Exit when Q is pressed

        except CvBridgeError, e:
            print(e)

	
    def image_callback(self,msg):
        # print('image_callback: ')
        
        try:
            self.frame   = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            # self.rvizframe   = self.frame
            # objectsphotos= self.crop_objects(self.frame)

            '''
            if self.color_track & self.Init_track:
                for id in range(len(self.target_strings)):
                    try:
                        fgame_mask = self.my_back_detectors[id].returnMask(self.frame, morph_opening=True, blur=True, kernel_size=5, iterations=2)
                    except:
                        rospy.sleep(1)
                    if(self.mask_analysers[id].returnNumberOfContours(frame_mask) > 0):
                        x_rect,y_rect,w_rect,h_rect = self.mask_analysers[id].returnMaxAreaRectangle(frame_mask)
                        x_center, y_center = self.mask_analysers[id].returnMaxAreaCenter(frame_mask)
                        # Adding noise to the coords
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
                        cv2.rectangle(self.frame, (x_rect,y_rect), (x_rect+w_rect,y_rect+h_rect), [255,255,0], 2) #BLUE rect

                        #Predict the position of the target
                        self.templete_trackers[id].predict(x_velocity=0, y_velocity=0, std=self.std)

                        #Drawing the particles.
                        self.templete_trackers[id].drawParticles(self.frame)

                        #Estimate the next position using the internal model
                        x_estimated, y_estimated, _, _ = self.templete_trackers[id].estimate()
                        cv2.circle(self.frame, (x_estimated, y_estimated), 3, [0,125,120], 5) #GREEN dot
                        self.templete_trackers[id].update(x_center, y_center)
                        self.templete_trackers[id].resample()

               #Update the filter with the last measurements
                # self.my_particle.update(x_center, y_center)

               #Resample the particles
                # self.my_particle.resample()
            '''
            #--------------------
            if self.Init_track:
                for bbox in self.bounding_boxes:
                    for id in range(len(self.target_strings)):
                        # if bbox.Class == self.target_strings[id]:

                        desired_head_pan= self.robot_pose[3]
                        if bbox.Class == 'bottle':
                            self.context_modes[id]=0
                            self.last_imagetime=rospy.get_time()
                            self.update_trackers(0,bbox.xmin,bbox.xmax,bbox.ymin,bbox.ymax)
                            box_center = Point((bbox.xmin+bbox.xmax)/2, (bbox.xmin+bbox.xmax)/2, self.target_z)
                            if box_center.x<200:
                                ee_desired_cmd=1
                                desired_head_pan = self.robot_pose[3]+0.3
                                rospy.loginfo("deisred_head : left")
                            elif box_center.x<100:
                                desired_head_pan = self.robot_pose[3]+0.6
                                rospy.loginfo("deisred_head : left")
                            elif box_center.x>450:
                                ee_desired_cmd=2
                                desired_head_pan = self.robot_pose[3]-0.3
                                rospy.loginfo("deisred_head : right")
                            elif box_center.x>560:
                                ee_desired_cmd=2
                                desired_head_pan = self.robot_pose[3]-0.55
                                rospy.loginfo("deisred_head : right")
                            else:
                                ee_desired_cmd=0
                                desired_head_pan= self.robot_pose[3]
                            
                        #publish the desired joint angles
                            desired_pan_joint = Float32()
                            desired_pan_joint.data= desired_head_pan
                            self.head_command_pub.publish(desired_pan_joint)

                cur_time = rospy.get_time()
                time_duration = cur_time -self.last_imagetime
                if (time_duration >5.0):
                    self.context_modes[id]=self.last_context_mode
                    



                            # self.update_trackers(id,bbox.xmin,bbox.xmax,bbox.ymin,bbox.ymax)
                                

                    # self.update_trackers(id,self.yolo_bbs[id].xmin, self.yolo_bbs[id].xmax, self.yolo_bbs[id].ymin, self.yolo_bbs[id].ymax)
                        # if self.infoarray.objectinfos[index].id == self.target_ids[id]:
                            # bbox = self.infoarray.objectinfos[index].bbox
                            # x_center=bbox.x+bbox.width/2
                            # y_center=bbox.y+bbox.height/2
                            # self.update_trackers(id,bbox.x,bbox.x+bbox.width,bbox.y,bbox.y+bbox.height)
            #--------------------------------------------

               #Writing in the output file
               # out.write(frame)
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))

            # winname="pf_test"
            '''
            cv2.namedWindow("image_window", cv2.WINDOW_NORMAL)
            cv2.imshow("image_window", self.frame)
            cv2.waitKey(1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                exit()
               # break #Exit when Q is pressed
            '''


        except CvBridgeError, e:
            print(e)

    # def update_infotrackers(self, idx, bbox_xmin, bbox_xmax, bbox_ymin, bbox_ymax):
        # x_meas = bbox_xmin+(bbox_xmax-bbox_xmin)/2
        # y_meas = bbox_ymin+(bbox_ymax-bbox_ymin)/2
        # self.trackers[idx].predict(x_velocity=0, y_velocity=0, std=self.std)
        # self.trackers[idx].drawParticles(self.rvizframe)
        # x_estimated, y_estimated, _, _ = self.trackers[idx].estimate()
        # cv2.circle(self.rvizframe, (x_estimated, y_estimated), 3, [255,0,0], 5) #GREEN dot
        # self.trackers[idx].update(x_meas, y_meas)
        # cv2.rectangle(self.rvizframe, (int(bbox_xmin),int(bbox_ymin)), (int(bbox_xmax),int(bbox_ymax)), [0,255,0], 2) #BLUE rect
        #

    def update_trackers(self, idx, bbox_xmin, bbox_xmax, bbox_ymin, bbox_ymax):
        x_meas = bbox_xmin+(bbox_xmax-bbox_xmin)/2
        y_meas = bbox_ymin+(bbox_ymax-bbox_ymin)/2
        self.trackers[idx].predict(x_velocity=0, y_velocity=0, std=self.std)
        self.trackers[idx].drawParticles(self.frame)
        x_estimated, y_estimated, _, _ = self.trackers[idx].estimate()
        cv2.circle(self.frame, (x_estimated, y_estimated), 3, [0,255,0], 5) #GREEN dot
        self.trackers[idx].update(x_meas, y_meas)
        cv2.rectangle(self.frame, (int(bbox_xmin),int(bbox_ymin)), (int(bbox_xmax),int(bbox_ymax)), [255,0,0], 2) #BLUE rect
        try:
            self.trackers[idx].resample()
        except: 
            rospy.loginfo("resample failed")

    def update_trackers_handimage(self, idx, bbox_xmin, bbox_xmax, bbox_ymin, bbox_ymax):
        x_meas = bbox_xmin+(bbox_xmax-bbox_xmin)/2
        y_meas = bbox_ymin+(bbox_ymax-bbox_ymin)/2
        self.trackers[idx].predict(x_velocity=0, y_velocity=0, std=self.std)
        self.trackers[idx].drawParticles(self.handframe)
        x_estimated, y_estimated, _, _ = self.trackers[idx].estimate()
        cv2.circle(self.handframe, (x_estimated, y_estimated), 3, [0,255,0], 5) #GREEN dot
        self.trackers[idx].update(x_meas, y_meas)
        cv2.rectangle(self.handframe, (int(bbox_xmin),int(bbox_ymin)), (int(bbox_xmax),int(bbox_ymax)), [255,0,0], 2) #BLUE rect
        try:
            self.trackers[idx].resample()
        except: 
            rospy.loginfo("resample failed")



    def visualizeparticles(self):
        
        cloudsetarray=[]
        for id in range(len(self.trackers3d)):
            cloud_sets=[]
            for x_particle, y_particle in self.trackers3d[id].particles.astype(float):
                cloud_sets.append([x_particle,y_particle,self.target_z])
                # rospy.loginfo("x : %.2lf, y : %.2lf ", x_particle, y_particle)

            cloudsetarray.append(cloud_sets)
        
        header=std_msgs.msg.Header()
        header.stamp=rospy.Time.now()
        header.frame_id='map'
        particle_pcl=pcl2.create_cloud_xyz32(header,cloudsetarray[0])
        # print particle_pcl.fields
        self.pcl_pub.publish(particle_pcl)


        # header=std_msgs.msg.Header()
        # header.stamp=rospy.Time.now()
        # header.frame_id='map'
        # particle_pcl2=pcl2.create_cloud_xyz32(header,cloudsetarray[1])
        # self.pcl_pub2.publish(particle_pcl2)
        

        # print "visualize detected"
        # rospy.loginfo("visualize particles")
        # rospy.loginfo("last statement in estimatefilter")

        #----------------visualize context particles------------------------------ 
        # cloud_sets2=[]
        # for x_particle2, y_particle2 in self.context_particle.particles.astype(float):
            # cloud_sets2.append([x_particle2,y_particle2,self.target_z])
           ## rospy.loginfo("x : %.2lf, y : %.2lf ", x_particle2, y_particle2)
        
        ## rospy.loginfo("length of context particle:%d ", len(cloud_sets2))
        # header=std_msgs.msg.Header()
        # header.stamp=rospy.Time.now()
        # header.frame_id='map'
        # header.frame_id='/world'
        # particle_pcl2=pcl2.create_cloud_xyz32(header,cloud_sets2)
        # self.pcl_context_pub.publish(particle_pcl2)
        #--------------visualize context particles------------------------------ 
  
    def Estimate_Filter3D_idx(self,idx):
        self.trackers3d[idx].predict(x_velocity=0.0, y_velocity=0.0, std=self.std3d)

    def Estimate_Filter3D_point(self,idx,point):
        self.trackers3d[idx].predict_point(point, std=self.std3d)
        self.trackers3d[idx].predict(x_velocity=0.0, y_velocity=0.0, std=self.std3d)

    def Estimate_Filter3D_pointset(self,idx,pointset):
        self.trackers3d[idx].predict_pointset(pointset, std=self.std3d)
        self.trackers3d[idx].predict(x_velocity=0.0, y_velocity=0.0, std=self.std3d)

    def Estimate_Filter3D(self):
        #Predict the position of the target
        for id in range(len(self.trackers3d)):
            if self.context_modes[id]==0: 
               self.Estimate_Filter3D_idx(id)
                # self.trackers3d[id].predict(x_velocity=0.01, y_velocity=0.01, std=self.std3d)
            elif self.context_modes[id]==1: # searching mode
                rospy.loginfo("searching mode for %s", self.target_strings[id])
                self.get_fov_outpoints()
                self.Estimate_Filter3D_pointset(id,self.get_fov_outpoints())
                #update no measurement
                for i in range(len(self.trackers3d[id].particles)):
                    x_particle = self.trackers3d[id].particles[i,0]
                    y_particle = self.trackers3d[id].particles[i,1]
                    if self.Is_inFOV(x_particle,y_particle, self.robot_pose)==True:
                        self.trackers3d[id].particles[i,0]=self.last_targetlocations[id].x+uniform(-0.05,0.05)
                        self.trackers3d[id].particles[i,1]=self.last_targetlocations[id].y+uniform(-0.05,0.05)

            elif self.context_modes[id]==2: # occ_with_identified object
                rospy.loginfo("occ_with identifiable object")
                self.Estimate_Filter3D_point(id,self.occ_point)
                self.last_context_mode=self.context_modes[id]

            elif self.context_modes[id]==3: # occ_with_nonidentified object mode
                rospy.loginfo("occ_with non-identifiable object")
                self.Estimate_Filter3D_point(id,self.last_targetlocations[id])
                self.last_context_mode=self.context_modes[id]
                for i in range(len(self.trackers3d[id].particles)):
                    x_particle = self.trackers3d[id].particles[i,0]
                    y_particle = self.trackers3d[id].particles[i,1]
                    if self.Is_inFOV(x_particle,y_particle, self.robot_pose)==True:
                        self.trackers3d[id].particles[i,0]=self.last_targetlocations[id].x+uniform(-0.05,0.05)
                        self.trackers3d[id].particles[i,1]=self.last_targetlocations[id].y+uniform(-0.05,0.05)


 
                #TODO: context!!!

            # x_estimated, y_estimated, _, _ = self.trackers3d[id].estimate()

        # est_pose=PoseStamped()
        # est_pose.pose=Pose(Point(x_estimated,y_estimated,0.5),Quaternion(0,0,0,1))
        # est_pose.header.stamp=rospy.Time.now()
        # est_pose.header.frame_id='map'
        # self.estimated_point_pub.publish(est_pose)

    def decompose_particles(self):
        #Decompose particles based on the field of view of sampled q_k
        cloud_sets3=[]
        avg_count=0
        self.s1_weights=[]
        # calculate the particle without FOV
        # obtain average locations of the paritlce that are not in FOV
        for id in range(len(self.trackers3d)):
            for i in range(len(self.trackers3d[id].particles)):
                x_particle = self.trackers3d[id].particles[i,0]
                y_particle = self.trackers3d[id].particles[i,1]
                if self.Is_inFOV(x_particle,y_particle,self.robot_pose)==False:
                    self.avg_x+=x_particle
                    self.avg_y+=y_particle
                    avg_count=avg_count+1;
                    cloud_sets3.append([x_particle,y_particle,0.9])
                    self.s1_weights.append(self.trackers3d[id].weights[i])

        #If most of partilces are visible, return
        if avg_count<30:
            return
        #for debug ------------------------------------------------
        # else:
            # rospy.loginfo("avg count not in FOV:%d", avg_count )
            # header=std_msgs.msg.Header()
            # header.stamp=rospy.Time.now()
            # header.frame_id='map'
            # particle_pcl3=pcl2.create_cloud_xyz32(header,cloud_sets3)
            # self.pcl_fov_pub.publish(particle_pcl3)
        #-----------------------------------------------------------
        
        #s1(not fov) particles
        self.s1_particles=np.empty((len(cloud_sets3),2))
        for i in range(len(cloud_sets3)):
            self.s1_particles[i,0]=cloud_sets3[i][0]
            self.s1_particles[i,1]=cloud_sets3[i][1]

        #Here we update positions 

        #get average position data from s1
        self.avg_x=self.avg_x/avg_count;
        self.avg_y=self.avg_y/avg_count;

        #calculate
        avg_pose=PoseStamped()
        avg_pose.pose=Pose(Point(self.avg_x,self.avg_y,self.target_z),Quaternion(0,0,0,1))
        avg_pose.header.stamp=rospy.Time.now()
        avg_pose.header.frame_id='map'
        self.avg_point_pub.publish(avg_pose)

        #update interior particles in my_particles
        # for id in range(len(self.trackers3d)):
            # if self.context_modes[id]==1: # searching mode
                # for i in range(len(self.trackers3d[id].particles)):
                    # x_particle = self.trackers3d[id].particles[i,0]
                    # y_particle = self.trackers3d[id].particles[i,1]
                    # if self.Is_inFOV(x_particle,y_particle, self.robot_pose)==True:
                        # self.trackers3d[id].particles[i,0]=self.last_targetlocations[id].x+uniform(-0.25,0.25)
                        # self.trackers3d[id].particles[i,1]=self.last_targetlocations[id].y+uniform(-0.25,0.25)

    def check_obsb_dynamic(self,pos_x,pos_y):
        if self.map_received==False:
           return False
        map_idx=self.Coord2CellNum(pos_x,pos_y)
        # rospy.loginfo("map idx : %d ", map_idx )

        if(self.dynamic_map.data[map_idx]>20):
            return True
        else:
            return False


    def check_obsb(self,pos_x,pos_y):
        #return True if point collides obstacles
        if self.map_received==False:
           return False


        map_idx=self.Coord2CellNum_STATIC(pos_x,pos_y)
        # rospy.loginfo("map idx : %d ", map_idx )

        if(self.static_map.data[map_idx]>20):
            return True
        else:
            return False

    def Coord2CellNum(self, pos_x, pos_y):
        #calculate dynamic map index number from position information
        target_Coord=[]

        reference_origin_x=self.dynamic_map.info.origin.position.x
        reference_origin_y=self.dynamic_map.info.origin.position.y

        temp_x = pos_x-reference_origin_x
        temp_y = pos_y-reference_origin_y

        target_Coord_x= int(temp_x/self.dynamic_map.info.resolution)
        target_Coord_y= int(temp_y/self.dynamic_map.info.resolution)

        index= target_Coord_x+self.dynamic_map.info.width*target_Coord_y
        return index

    def Coord2CellNum_STATIC(self, pos_x, pos_y):
        #calculate static map index number from position information
        target_Coord=[]

        reference_origin_x=self.static_map.info.origin.position.x
        reference_origin_y=self.static_map.info.origin.position.y

        temp_x = pos_x-reference_origin_x
        temp_y = pos_y-reference_origin_y

        target_Coord_x= int(temp_x/self.static_map.info.resolution)
        target_Coord_y= int(temp_y/self.static_map.info.resolution)

        index= target_Coord_x+self.static_map.info.width*target_Coord_y
        return index

    def getlinevalue_int(self, line_type, input_x, input_y, 
                        int_point1, int_point2,  robot_pos):
        
        slope1 = (int_point1.y-robot_pos[1])/(int_point1.x-robot_pos[0])
        slope2 = (int_point2.y-robot_pos[1])/(int_point2.x-robot_pos[0])
        theta_1 = math.atan(slope1)
        theta_2 = math.atan(slope2)
        # rospy.loginfo("theta1 : %.2lf, theta2 : %.2lf ", theta_1, theta_2)
        mode=0
        coeff_sign=-1.0

        if (theta_1 < -math.pi/2.0) & (theta_2 > -math.pi/2.0):
            mode=1
        elif (theta_1 < math.pi/2.0) & (theta_2 > math.pi/2.0):
            mode=2
        elif (theta_1 < -math.pi/2.0) & (theta_2 < -math.pi/2.0):
            mode=5
        elif  (theta_2 < -math.pi/2.0):
            mode=3
        elif (theta_1 > math.pi/2) & (theta_2 > math.pi/2.0):
            mode=4

        if line_type==1:
            slope=slope1
            if mode==0:
               coeff_sign=-1.0
            elif mode==1:
               coeff_sign=1.0
            elif mode==2:
               coeff_sign=-1.0  
            elif mode==4:
               coeff_sign=1.0 
            elif mode==5:
               coeff_sign=1.0

        elif line_type ==2:
            slope=slope2
            if mode==1:
                coeff_sign=1.0;
            elif mode==0:
                coeff_sign=1.0 
            elif mode==3:
                coeff_sign=1.0
        else:
            rospy.loginfo("linetype is wrong")

        res= slope*(input_x-robot_pos[0])+robot_pos[1]-input_y;

        if res*coeff_sign > 0 :
            return True
        else:
            return False



    def getlinevalue(self, line_type, input_x, input_y,robot_pos):
        head_angle = robot_pos[2]
        theta_1= head_angle-29*math.pi/180.0
        theta_2= head_angle+29*math.pi/180.0
        # theta_2= self.robot_pose[2]+self.robot_pose[3]+29*math.pi/180.0
        slope1 = math.tan(theta_1)
        slope2 = math.tan(theta_2)
        mode=0
        
        coeff_sign=-1.0

        if (theta_1 < -math.pi/2.0) & (theta_2 > -math.pi/2.0):
            mode=1
        elif (theta_1 < math.pi/2.0) & (theta_2 > math.pi/2.0):
            mode=2
        elif (theta_1 < -math.pi/2.0) & (theta_2 < -math.pi/2.0):
            mode=5
        elif  (theta_2 < -math.pi/2.0):
            mode=3
        elif (theta_1 > math.pi/2) & (theta_2 > math.pi/2.0):
            mode=4

        if line_type==1:
            slope=slope1
            if mode==0:
               coeff_sign=-1.0
            elif mode==1:
               coeff_sign=1.0
            elif mode==2:
               coeff_sign=-1.0  
            elif mode==4:
               coeff_sign=1.0 
            elif mode==5:
               coeff_sign=1.0

        elif line_type ==2:
            slope=slope2
            if mode==1:
                coeff_sign=1.0;
            elif mode==0:
                coeff_sign=1.0 
            elif mode==3:
                coeff_sign=1.0
        else:
            rospy.loginfo("linetype is wrong")

        res= slope*(input_x-robot_pos[0])+robot_pos[1]-input_y;

        if res*coeff_sign > 0 :
            return True
        else:
            return False


    def Is_inFOV_point_occ(self, point_x,point_y, test_robot_pos,occ_pos):
        #return True if point is in FOV
        #Use image plane
        test_robot_pose=np.empty((3,1))
        test_robot_pose[0]=test_robot_pos[0]
        test_robot_pose[1]=test_robot_pos[1]
        test_robot_pose[2]=test_robot_pos[2]
        
        if len(test_robot_pos)>3:
           test_robot_pose[2]=self.robot_pose[2]+self.robot_pose[3]

        #check if occ is in fov
        if self.Is_inFOV_point(occ_pos,test_robot_pos):
           # rospy.loginfo("occ in fov")

           res1=self.getlinevalue(1,point_x, point_y,test_robot_pose)
           res2=self.getlinevalue(2,point_x, point_y,test_robot_pose)

           if res1 & res2:
                # rospy.loginfo("point ( %.3lf, %.2lf) is also in fov",point_x, point_y)
                point_inFOV=True
           else:
                return False

           #find intersection point with 
           origin_robot = Point(test_robot_pos[0], test_robot_pos[1],0.0)
           tangentset = tangent_point_circle_extpoint(occ_pos, OCC_RADIUS, origin_robot)


           test_point1 = Point(tangentset[0].x,tangentset[0].y,self.target_z) #flt
           test_point2 = Point(tangentset[1].x,tangentset[1].y,self.target_z) #frt

           tan_point1 = PointStamped()
           tan_point2 = PointStamped()
           tan_point1.point=test_point1
           tan_point1.header.frame_id = 'map'
           tan_point1.header.stamp= rospy.Time.now()
           tan_point2.point=test_point2
           tan_point2.header.frame_id = 'map'
           tan_point2.header.stamp= rospy.Time.now()
           self.tangent_point1_pub.publish(tan_point1)
           self.tangent_point2_pub.publish(tan_point2)
           # rospy.sleep(0.1)

           res1=self.getlinevalue_int(1,point_x, point_y,test_point1, test_point2, test_robot_pose)
           res2=self.getlinevalue_int(2,point_x, point_y,test_point1, test_point2, test_robot_pose)

           if res1 & res2:
               return False
           else:
                # rospy.loginfo("x : %.2lf, y : %.2lf ", x_particle, y_particle)
               return True

        else:
           return self.Is_inFOV(point_x,point_y,test_robot_pos)
            


    def Is_inFOV_point(self, point, test_robot_pos):
        #return True if point is in FOV
        test_robot_pose=np.empty((3,1))
        test_robot_pose[0]=test_robot_pos[0]
        test_robot_pose[1]=test_robot_pos[1]
        test_robot_pose[2]=test_robot_pos[2]
        
        if len(test_robot_pos)>3:
            test_robot_pose[2]=self.robot_pose[2]+self.robot_pose[3]

        res1=self.getlinevalue(1,point.x, point.y,test_robot_pose)
        res2=self.getlinevalue(2,point.x, point.y,test_robot_pose)

        if res1 & res2:
            return True
        else:
            return False
    
    def Is_inFOV(self, point_x, point_y,test_robot_pos):
        #return True if point is in FOV

        test_robot_pose=np.empty((3,1))
        test_robot_pose[0]=test_robot_pos[0]
        test_robot_pose[1]=test_robot_pos[1]
        test_robot_pose[2]=test_robot_pos[2]

        if len(test_robot_pos)>3:
            test_robot_pose[2]=self.robot_pose[2]+self.robot_pose[3]

        # rospy.loginfo("test_robot_pose: x: %.3lf, y: %.3lf, z:%.3lf", test_robot_pose[0],test_robot_pose[1],test_robot_pose[2])

        res1=self.getlinevalue(1,point_x, point_y,test_robot_pose)
        res2=self.getlinevalue(2,point_x, point_y,test_robot_pose)
        if res1 & res2:
            return True
        else:
            return False

    def filter_by_fov(self):
        #based on projected_map from octomap
        #remove partilces from partilces false-postive
        if self.map_received:
            for id in range(len(self.trackers3d)):
                for i in range(len(self.trackers3d[id].particles)):
                    x_particle = self.trackers3d[id].particles[i,0]
                    y_particle = self.trackers3d[id].particles[i,1]
                    if self.check_obsb(x_particle, y_particle)==True:
                        self.trackers3d[id].particles[i,0]=self.last_targetlocations[id].x+3*uniform(-0.1,0.1)
                        self.trackers3d[id].particles[i,1]=self.last_targetlocations[id]+3*uniform(-0.1,0.1)



    def filter_by_Occgrids(self):
        #based on projected_map from octomap
        #remove partilces from partilces false-postive
        if self.map_received:
            for id in range(len(self.trackers3d)):
                for i in range(len(self.trackers3d[id].particles)):
                    x_particle = self.trackers3d[id].particles[i,0]
                    y_particle = self.trackers3d[id].particles[i,1]
                    if self.check_obsb(x_particle, y_particle)==True:
                        self.trackers3d[id].particles[i,0]=self.last_targetlocations[id].x+3*uniform(-0.1,0.1)
                        self.trackers3d[id].particles[i,1]=self.last_targetlocations[id]+3*uniform(-0.1,0.1)

    def get_travelcost(self, robot_config):

        #should get current configuration 
        #get difference between two configurations
        coeff=[2.0, 2.0 , 1.0]
        diff=0.0
        for idx in range(len(robot_config)):
            diff+=coeff[idx]*math.pow(self.robot_pose[idx]-robot_config[idx],2)

        # rospy.loginfo(diff)
        return diff
        #get yaw angle from quarternion
 

    def generate_q_samples(self, num):
        sample_num=num
        y_distance_bound=1.2;
        x_distance_bound=0.4;
        # rospy.loginfo("robot pose_theta: %.3lf, robot pose_y: %.3lf", self.robot_pose[2], self.robot_pose[3])
        camera_ori = self.robot_pose[2]+self.robot_pose[3]

        # rospy.loginfo("camera_ori: %.3lf", camera_ori)
        while camera_ori>2*math.pi:
            camera_ori-=2*math.pi

        while camera_ori<-2*math.pi:
            camera_ori+=2*math.pi
        
        # rospy.loginfo("camera_ori: %.3lf", camera_ori)

        #generate random position from robot
        self.meas_samples=np.empty((num,3))
        self.meas_samples[:,0]=np.random.uniform(self.robot_pose[0]-2*x_distance_bound, self.robot_pose[0]+0.5*x_distance_bound,num)
        self.meas_samples[:,1]=np.random.uniform(self.robot_pose[1]-y_distance_bound, self.robot_pose[1]+y_distance_bound,num)
        random_theta=camera_ori+np.random.uniform(-math.pi/3,math.pi/3,num)

        # print "camera ori", camera_ori
        # print "math.pi/2",math.pi/2
        # print "random_theta",random_theta
        self.meas_samples[:,2]=random_theta
        # self.meas_samples[:,2]=np.random.uniform(camera_ori-math.pi/2,camera_ori+math.pi/2,num)

        #check obstacles 
        for i in range(len(self.meas_samples)):
            x_pos=self.meas_samples[i,0]
            y_pos=self.meas_samples[i,1]
            count=0
            while self.check_obsb(x_pos,y_pos)==True | count <500:
                x_pos=np.random.uniform(self.robot_pose[0]-2*x_distance_bound, self.robot_pose[0]+2*x_distance_bound,1)
                y_pos=np.random.uniform(self.robot_pose[1]-2*y_distance_bound, self.robot_pose[1]+2*y_distance_bound,1)
                count=count+1
            self.meas_samples[i,0]=x_pos
            self.meas_samples[i,1]=y_pos



        #generate pose_array_from sampled poses
        meas_sampleposes=PoseArray()
        for i in range(len(self.meas_samples)):
            test_quat= quaternion_from_euler(0,0,self.meas_samples[i,2])
            meas_sample_pose=Pose(Point(self.meas_samples[i,0],self.meas_samples[i,1],self.target_z),Quaternion(test_quat[0],test_quat[1],test_quat[2],test_quat[3]))
            meas_sampleposes.poses.append(meas_sample_pose)
        
        #publish posearray message
        meas_sampleposes.header.stamp=rospy.Time.now()
        meas_sampleposes.header.frame_id='map'
        self.sample_meas_pub.publish(meas_sampleposes)


    def evaluate_q_samples(self):
        # self.get_expected_entropy()
        #for each fov, calculated expected number of samples 
        # expected_entropy=np.empty((len(self.meas_samples),1))
        is_occfovclouds=False
        expected_entropy=[]
        particle_countset=[]
        for i in range(len(self.meas_samples)):
            entropy_sum=0.0
            particle_count=0
            # rospy.loginfo("meas_robot_pose: x: %.3lf, y: %.3lf, z:%.3lf", self.meas_samples[i][0],self.meas_samples[i][1],self.meas_samples[i][2])
            #In the case that 
            if self.context_modes[0]<2:
                # rospy.loginfo("tracking or searching")
                for j in range(len(self.trackers3d[0].particles)):
                    particle_x=self.trackers3d[0].particles[j,0]
                    particle_y=self.trackers3d[0].particles[j,1]
                    if self.Is_inFOV(particle_x, particle_y,self.meas_samples[i])==True:
                    # print "self.meas_samples",self.meas_samples[i]
                    # rospy.loginfo("in field of view")
                        entropy_sum-=self.trackers3d[0].weights[j]*np.log2(self.trackers3d[0].weights[j])
                        particle_count+=1
            else:
                #TODO
                # rospy.loginfo("occlusion form others")
                occfovclouds=[]
                for j in range(len(self.trackers3d[0].particles)):
                    particle_x=self.trackers3d[0].particles[j,0]
                    particle_y=self.trackers3d[0].particles[j,1]
                    if self.Is_inFOV_point_occ(particle_x, particle_y,self.meas_samples[i],self.occ_point)==True:
                    # print "self.meas_samples",self.meas_samples[i]
                    # rospy.loginfo("in field of view")
                        is_occfovclouds=True
                        occfovclouds.append([particle_x, particle_y, self.target_z])
                        entropy_sum-=self.trackers3d[0].weights[j]*np.log2(self.trackers3d[0].weights[j])
                        particle_count+=1

            # rospy.loginfo("partilce counts : %d", particle_count)
            #TODO: add traveling cost
            entropy_sum=3*entropy_sum
            traveling_cost = self.get_travelcost(self.meas_samples[i])
            entropy_sum-=0.5*traveling_cost
            expected_entropy.append(entropy_sum)
            # rospy.loginfo("ettropy_sum: %.3lf", entropy_sum)
            particle_countset.append(particle_count)

        if is_occfovclouds:
            current_time = rospy.get_time()
            if (current_time-self.last_occfovtime)>10.0:
                rospy.loginfo("occpartice sizes = %d", len(occfovclouds))
                header=std_msgs.msg.Header()
                header.stamp=rospy.Time.now()
                header.frame_id='map'
                occparticle_pcl=pcl2.create_cloud_xyz32(header,occfovclouds)
                self.last_occfovtime = rospy.get_time()
                self.occpcl_pub.publish(occparticle_pcl)
                    
        # print "expected_entropy", expected_entropy
        sorted_entropy = sorted(((v,i) for i, v in enumerate(expected_entropy)),reverse=True)
        # sorted_entropy = sorted(((v,i) for i, v in enumerate(particle_countset)),reverse=True)
        # rospy.loginfo("maximum value: %.4lf,  index: %d", sorted_entropy[0][0], sorted_entropy[0][1])
        #find the best index
        max_idx=sorted_entropy[0][1]
        # print particle_countset
        rospy.loginfo("best index particle number: %d", particle_countset[max_idx])
        rospy.loginfo("best cost function value: %.2lf", sorted_entropy[0][0])

        selected_pose=PoseStamped()
        test_quat= quaternion_from_euler(0,0,self.meas_samples[max_idx,2])
        selected_pose.pose=Pose(Point(self.meas_samples[max_idx][0],self.meas_samples[max_idx][1],self.target_z),Quaternion(test_quat[0],test_quat[1],test_quat[2],test_quat[3]))
        selected_pose.header.stamp=rospy.Time.now()
        selected_pose.header.frame_id='map'
        self.opt_pose_pub.publish(selected_pose)
        self.sm_feedback.opt_pose= selected_pose

        '''
        desird_head_angle = self.meas_samples[max_idx,2]
        #this desired_head_angle includes base+_head
        desird_head_angle = desird_head_angle-self.robot_pose[2]
        diff_angle = desird_head_angle -self.robot_pose[3];
        if diff_angle>1.0:
            diff_angle=0.5
        elif diff_angle<-1.0:
            diff_angle=-0.5
        elif abs(diff_angle)<0.4:
            return
        
        #publish the desired joint angles
        # desired_pan_joint = Float32()
        # desired_pan_joint.data= self.robot_pose[3]+diff_angle*0.6
        # self.head_command_pub.publish(desired_pan_joint)
        '''

        distance =0.0
        distance +=math.pow(self.robot_pose[0]-selected_pose.pose.position.x,2)
        distance +=math.pow(self.robot_pose[1]-selected_pose.pose.position.y,2)
        distance  =math.sqrt(distance)
        # distance = np.linalg.norm(self.particles - position, axis=1)
        #call action client
        if self.navi_mode==1:
            if distance>0.5:
                self.navigation_action(selected_pose.pose.position.x, selected_pose.pose.position.y, desird_head_angle)
                self.navi_mode=0
            else:
                rospy.loginfo("distance limit")


    def listener(self,wait=0.0):
        # rospy.spin() 

        while not rospy.is_shutdown():
            #should estimate without measurement
            if self.Init_track:
                self.Estimate_Filter3D()
                self.decompose_particles()
		self.get_fov_outpoints()
                # self.filter_by_Occgrids()
                self.generate_q_samples(self.meas_sample_num)
                self.evaluate_q_samples()
                self.visualizeparticles()
                # cur_time=rospy.get_time()


                # for id in range(len(self.last_observations)):
                    # duration = cur_time -self.last_observations[id]
                    # if duration > 10.0:
                        # self.context_modes[id]=1 #set to searching mode

                #navigation modes
                # navi_duration = cur_time -self.last_navitime
                # rospy.loginfo("duration : %lf", duration)
                # if navi_duration>15 :
                    # self.navi_mode=1
                # else:
                    # self.navi_mode=0
            
            # rospy.spinOnce()
            rospy.Rate(1).sleep()



if __name__ == '__main__':
        rospy.init_node('multi_particle_filte')
	# print("Initialize node")
        tracker_manager = ObjectTracker('multi_tracker_action')
        rospy.loginfo("multi_tracker actioncreated")
	tracker_manager.listener()	

