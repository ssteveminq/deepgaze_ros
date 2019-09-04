#!/usr/bin/env python
#!/usr/bin/python

# ROS version written by Minkyu Kim
# import roslib;
import rospy
import sys
import time
# print(sys.path)
import cv2
import actionlib
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from deepgaze_ros.msg import *
from deepgaze_ros.msg import PersonTrackAction
import numpy as np
import argparse
from sensor_msgs.msg import Image
from openpose_ros_wrapper_msgs.msg import PersonsArray
from openpose_ros_wrapper_msgs.msg import BodyPartDetection
from openpose_ros_wrapper_msgs.msg import HandDetections
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from human_tracker.srv import GetHandPositions
import select, termios, tty
from sensor_msgs.msg import Image
from std_msgs.msg import String
from os import listdir
import os

_MAP_TF='map'
_SENSOR_TF ='head_rgbd_sensor_rgb_frame'
CONFIDENCE_THRESHOLD=0.35
GETHAND_SRV='/openpose/hand_positions'
TRJ_SIZE=100
DrawTrj=True
showTrue=False

class PersonTracker(object):
    def __init__(self, name, wait=0.0):

        #ROS action started
        self._action_name= name
        self._as = actionlib.SimpleActionServer(self._action_name, deepgaze_ros.msg.MultiTrackAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        self.wrist_index=[4,7]
        # self.wrist_index=[7]
        self.sample_trj={"left_arm":[], "right_arm":[]}
        self.leftarm=[]
        self.righttarm=[]

        self.gethand_client = rospy.ServiceProxy(GETHAND_SRV, GetHandPositions)

        self.bridge = CvBridge()
        openpose_topic = "/openpose/pose"
        rospy.Subscriber(openpose_topic, PersonsArray, self.openpose_callback)
        image_topic = 'hsrb/head_rgbd_sensor/rgb/image_rect_color'
        rospy.Subscriber(image_topic, Image, self.image_callback)

        openpose_hands_topic = "/openpose/hands"
        rospy.Subscriber(openpose_hands_topic, HandDetections, self.openpose_hand_callback)

        self.lefthand_point_pub=rospy.Publisher("/righthand_3d",PointStamped,queue_size=30)
        self.righthand_point_pub=rospy.Publisher("/lefthand_3d",PointStamped,queue_size=30)

        # rospy.spin()

    def openpose_hand_callback(self,msg):

        output_rightpoint=PointStamped()
        output_leftpoint=PointStamped()
        output_rightpoint.header.stamp=rospy.Time.now()
        output_rightpoint.header.frame_id = _SENSOR_TF 
        output_leftpoint.header.stamp=msg.header.stamp
        output_leftpoint.header.frame_id = _SENSOR_TF 

        if len(msg.hands)>0:
            right_hand_point = msg.hands[0].right_hand
            left_hand_point = msg.hands[0].left_hand
            output_rightpoint.point=right_hand_point 
            output_leftpoint.point=right_hand_point 

            self.righthand_point_pub.publish(output_rightpoint)
            self.lefthand_point_pub.publish(output_leftpoint)

            # if right_hand_point !=None:

                # listener = tf.TransformListener()
                # listener.waitForTransform(_SENSOR_TF, _MAP_TF, rospy.Time(0),rospy.Duration(3.0))

                # right_hand=PointStamped()
                # right_hand.header.frame_id = _SENSOR_TF
                # right_hand.header.stamp =rospy.Time(0)
                # right_hand.point=right_hand_point 
                # output_rightpoint=listener.transformPoint(_MAP_TF, right_hand)
                # self.righthand_point_pub.publish(output_rightpoint)


            # if left_hand_point !=None:
                # left_hand=PointStamped()
                # left_hand.header.frame_id = _SENSOR_TF
                # left_hand.header.stamp =rospy.Time(0)
                # left_hand.point=left_hand_point 
                # output_leftpoint=listener.transformPoint(_MAP_TF, left_hand)
                # self.lefthand_point_pub.publish(output_leftpoint)



    def extract_3darm_callback(self):

        IsWrist=False
        # res_=self.gethand_client()
        # print(res_)

    def openpose_callback(self,msg):
        # rospy.loginfo("openpose_callback")
        x_coord=None
        y_coord=None

        for index in self.wrist_index:
            if msg.persons.persons[0].body_part[index].confidence>CONFIDENCE_THRESHOLD:
               x_coord=msg.persons.persons[0].body_part[index].x
               y_coord=msg.persons.persons[0].body_part[index].y
            elif msg.persons.persons[0].body_part[index-1].confidence>CONFIDENCE_THRESHOLD:
               x_coord=msg.persons.persons[0].body_part[index-1].x
               y_coord=msg.persons.persons[0].body_part[index-1].y
            else:
                continue

            # rospy.loginfo("(x,y): :"+str([x_coord,y_coord]))

            if index>5:
                if len(self.sample_trj["left_arm"])<TRJ_SIZE:
                    self.sample_trj["left_arm"].append([x_coord,y_coord])
                else:
                    self.sample_trj["left_arm"].pop(0)
                    self.sample_trj["left_arm"].append([x_coord,y_coord])

                   # cv2.line(self.cv_image,(ocx,ocy),(x_coord,y_coord), (251,255,25),5 )
            else:
                if len(self.sample_trj["right_arm"])<TRJ_SIZE:
                    self.sample_trj["right_arm"].append([x_coord,y_coord])
                else:
                    self.sample_trj["right_arm"].pop(0)
                    self.sample_trj["right_arm"].append([x_coord,y_coord])


        # rospy.loginfo("length of letft arm :"+str(len(self.sample_trj["left_arm"])))
        if len(self.sample_trj["left_arm"])>1:
            x_coord2= self.sample_trj["left_arm"][-1][0]
            y_coord2=self.sample_trj["left_arm"][-1][1]

            # rospy.loginfo("rect(x,y): :"+str([x_coord2,y_coord2]))
            cv2.rectangle(self.cv_image, (x_coord2-8, y_coord2-8), (x_coord2+8,y_coord2+5), [255,125,0], 2) #BLUE rect
         
        if len(self.sample_trj["right_arm"])>1:
            x_coord2= self.sample_trj["right_arm"][-1][0]
            y_coord2=self.sample_trj["right_arm"][-1][1]

            cv2.rectangle(self.cv_image, (x_coord2-8, y_coord2-8), (x_coord2+8,y_coord2+5), [255,0,255], 2) #BLUE rect

        # if DrawTrj:
            # for i in range(len(self.sample_trj["right_arm"])):
                # coordinates = self.sample_trj["right_arm"][i]

                # cx=coordinates[0] 
                # cy=coordinates[1] 

                # if i==0:
                    # ocx=cx
                    # ocy=cy
                    # continue
                # else:
                    # cv2.line(self.cv_image,(ocx,ocy),(cx,cy), (251,255,25),5 )
                    # ocx=cx
                    # ocy=cy

        if showTrue:
            cv2.imshow('image_window',self.cv_image)
            key= cv2.waitKey(1) &0xFF

    def execute_cb(self, goal):
        rospy.loginfo("target number: id :%d", goal.target_nums)

    def image_callback(self,msg):
        # print('image_callback')
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
            self.cv_image= self.bridge.imgmsg_to_cv2(msg,"bgr8")
        except CvBridgeError, e:
            # print("hello2")
            print(e)
        # roslogprint("get frames")
        #yolo
        '''
        # (H, W) = frame.shape[:2]
        # ln = net.getLayerNames()
        # ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        # blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416),
                    # swapRB=True, crop=False)
        # net.setInput(blob)
        # start = time.time()
        # layerOutputs = net.forward(ln)
        # end = time.time()

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

        '''

        # if showTrue:
            # cv2.imshow('image_window',self.cv_image)
            # key= cv2.waitKey(1) &0xFF
        # cv2.imshow("image_window", frame)
        # cv2.imshow('Mask', frame_mask) #show on window
        # cv2.waitKey(1)
            # if cv2.waitKey(3) & 0xFF == ord('q'):
                # break #Exit when Q is pressed

    def listener(self,wait=0.0):
        # while not rospy.is_shutdown():
        rospy.spin() 


if __name__ == '__main__':
        rospy.init_node('person_tracker_actionserver')
        # print("Initialize node")
        tracker_manager = PersonTracker("person_tracker_action")
        rospy.loginfo("person_tracker action server created")
        tracker_manager.listener()

