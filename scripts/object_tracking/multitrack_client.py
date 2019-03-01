#!/usr/bin/python
# -*- coding: utf-8 -*-
import hsrb_interface
from hsrb_interface import geometry
import rospy
import sys
import tf
import actionlib
import geometry_msgs.msg
from deepgaze_ros.msg import *


def mains():
    # Initialize
    client = actionlib.SimpleActionClient('multi_tracker_action',deepgaze_ros.msg.MultiTrackAction)
    client.wait_for_server()
    goal = deepgaze_ros.msg.MultiTrackGoal()
    goal.idset.data.append(0)
    # goal.idset.data.append(1)
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("start action")
        
if __name__ == '__main__':
    rospy.init_node('multi_track_client')
    mains()
