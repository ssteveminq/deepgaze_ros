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
    client = actionlib.SimpleActionClient('state_machine',deepgaze_ros.msg.ModeConverterAction)
    client.wait_for_server()
    rospy.loginfo("found action server")
    goal = deepgaze_ros.msg.ModeConverterGoal()
    # goal.target_labels.append('bottle')
    # goal.target_labels.append('cup')
    # goal.idset.data.append(1)
    client.send_goal(goal, done_cb=None,active_cb=None,feedback_cb=feedback_Cb)
    client.wait_for_result()
    rospy.loginfo("start action")

def feedback_Cb(msg):
    rospy.loginfo("feedback callback")
    print msg
        
if __name__ == '__main__':
    rospy.init_node('multi_track_client')
    mains()
