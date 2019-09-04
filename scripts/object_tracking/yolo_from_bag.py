#!/usr/bin/python
 
# Extract images from a bag file.
 
#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
print(cv2.__version__)
import os
import argparse
from sensor_msgs.msg import Image
# import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from pointcloud_processing_msgs.msg import ObjectInfo
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import time

SURVIVAL_FRAME= 150;
TEMP_OCC_TIME = 1.5;
OVERLAP_THRESHOLD=0.6;
QUEUE_SIZE = 10;
target_class=['person', 'bottle', 'cup', 'cellphone', 'cell phone']

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
 
# Reading bag filename from command line or roslaunch parameter.
#import os
#import sys
 
DrawTrj=True
saveTrue=False
showTrue=True
rgb_path = '/home/mk/workspaces/temp_data/'
depth_path= '/home/mk/workspaces/temp_data/'

class Obj_trajectories():
    def __init__(self,name_):
        self.name=name_
        trajectories=[]


class Process_Image():
    def __init__(self):
        self.bridge = CvBridge()
        self.obj_array=[]
        print("hello")
        # boxes = []
        # confidences = []
        # classIDs = []
        num_frame=0
        frame_iter=0

        trajectories=[]
        self.sample_trj={"cell phone":[], "person":[], "cup":[], "bottle":[]}
        # sample_trj["cell phone"].append(3.2)

        # print(sample_trj["cell phone"])
        # input("enter to continue")

        #test for istarget
        # wow="chair"
        # res=self.IsTarget(wow)

        with rosbag.Bag('/home/mk/workspaces/sim_ws/src/freenect_stack/freenect_launch/data_storage/test.bag', 'r') as bag: #bag file to be read;

            #check for total frames
            for topic,msg,t in bag.read_messages():
                num_frame=num_frame+1
            print("total frames: "+str(num_frame))
            
            for topic,msg,t in bag.read_messages():

                boxes = []
                confidences = []
                classIDs = []
                frame_iter=frame_iter+1

                timestr = "%.3f" %  msg.header.stamp.to_sec()
                # print(timestr)
                # print(t)
                #rgb imagE
                if frame_iter<1850 or frame_iter>3000:
                    continue
                # print("frame_iter"+str(frame_iter))

                if topic == "/camera/rgb/image_rect_color": #topic of the image;
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")

                    except CvBridgeError as e:
                        print(e)
                    timestr = "%.6f" %  msg.header.stamp.to_sec()

                    #yolo
                    (H, W) = cv_image.shape[:2]
                    ln = net.getLayerNames()
                    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
                    blob = cv2.dnn.blobFromImage(cv_image, 1 / 255.0, (416, 416),
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
                            if self.IsTarget(LABELS[classID])==False:
                                continue

                            

                        # filter out weak predictions by ensuring the detected
                        # probability is greater than the minimum probability
                            if confidence > args["confidence"]:
                                # scale the bounding box coordinates back relative to the
                        # size of the image, keeping in mind that YOLO actually
                        # returns the center (x, y)-coordinates of the bounding
                        # box followed by the boxes' width and height
                                box = detection[0:4] * np.array([W, H, W, H])
                                (centerX, centerY, width, height) = box.astype("int")
                                self.sample_trj[LABELS[classID]].append([centerX,centerY])


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
                            cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
                            text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
                            cv2.putText(cv_image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, color, 2)
                    if DrawTrj:
                        for i in range(len(self.sample_trj["cell phone"])):
                            coordinates = self.sample_trj["cell phone"][i]
                            
                            cx=coordinates[0] 
                            cy=coordinates[1] 
                            
                            if i==0:
                                ocx=cx
                                ocy=cy
                                continue
                            else:
                                cv2.line(cv_image,(ocx,ocy),(cx,cy), (251,255,25),5 )
                                ocx=cx
                                ocy=cy
                            # print(coordinates)

                    # %.6f means that there are 6 digits after the decimal point, which can be modified according to the accuracy;
                    if saveTrue:
                        image_name = timestr+ ".png" #Image Naming: Timestamp.png
                        cv2.imwrite(rgb_path + image_name, cv_image) #Save;

                    if frame_iter%100==0:
                        cv2.destroyAllWindows()
                    if showTrue:
                        cv2.imshow('image_window',cv_image)

                        key= cv2.waitKey(1) &0xFF

                elif topic == "/camera/depth/image_rect": #depthtopic;
                    try:
                        depth_image = self.bridge.imgmsg_to_cv2(msg,"passthrough")
                        # depth_image = self.bridge.imgmsg_to_cv2(msg,"32FC1")
                        # print("succedded")
                    except CvBridgeError as e:
                        print(e)

                    depth_array=np.array(depth_image,dtype=np.float32)
                    cv2.normalize(depth_array,depth_array,0,1,cv2.NORM_MINMAX)
                    (H, W) = depth_image.shape[:2]
                    # print("depth height & W)"+str(H)+", "+ str(W))

                    # cv2.normalize(depth_image,depth_image,0,1,cv2.NORM_MINMAX)
                    # gray =cv2.cvtColor(depth_image,cv2.COLOR_BGR2GRAY)

                    if saveTrue:
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        image_name = timestr+ "_depth.png" #Image Naming: Timestamp.png
                        cv2.imwrite(depth_path + image_name, depth_image) #Save;


                # elif topic == "/camera/depth_registered/points": #pointcloudstopic;
                    # print("pcl points")

                    # points_list = []

                    # for data in pc2.read_points(msg, skip_nans=True):
                        # points_list.append([data[0], data[1], data[2], data[3]])
                        # print("pcl data:"+str(data[0])+"," +str(data[1])+","+str(data[2])+","+str(data[3]))

                    # pcl_data = pcl.PointCloud_PointXYZRGB()
                    # pcl_data.from_list(points_list)


                    # timestr = "%.6f" %  msg.header.stamp.to_sec()
                    # laser_data_name = timestr + ".txt"
                    # laser_data_path = os.path.join(laser_path, laser_data_name)
                    # with open(laser_data_path, "w") as f:
                # print("-----------")
                # print(self.sample_trj)
        # self.plot_trajectories("cell phone")

                # num_boxes = len(boxes)
                # for i in range(len(boxes)):
                    # text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
                    # print(text)
                    # print(text"class:"+str(LABELS[classIDs[i]])+", confidence:"+str(confidences[i]))
    def IsTarget(self, _classname):
        for classname in target_class:
            if classname ==_classname:
                return True

        return False


    def plot_trajectories(self, classname):

        us=[]
        vs=[]
        for i in range(len(self.sample_trj[classname])):
            coordinates = self.sample_trj[classname][i]
            print(coordinates)
            us.append(coordinates[0])
            vs.append(coordinates[1])

        plt.plot(us,vs,'b')
        plt.show()


if __name__ == '__main__':
    try:
        image_creator = Process_Image()
    except rospy.ROSInterruptException:
        pass
