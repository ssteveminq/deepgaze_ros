#!/usr/bin/env python

import actionlib
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from deepgaze_ros.msg import ModeConverterAction, ModeConverterGoal
from std_msgs.msg import Int32MultiArray
from tmc_msgs.msg import BatteryState
from hsrb_interface import Robot
from hsrb_interface import exceptions
# from villa_manipulation.msg import *
# import roslib
import rospy
import smach
import smach_ros
from smach import State
import tf.transformations

import csv
import numpy as np
import time


def get_action():

    # if cmd_idx==len(desired_states):
    desired_state=get_policy()

    if desired_state == 0:
        output_state = 'Go_Tracking'
    else:
        output_state = 'Go_Searching'
    # elif desired_state == 1:
        # output_state = 'Go_Searching'
    # else:  
        # print "desired state out of bounds"
        # output_state = 'end_demo'
    return desired_state, output_state

def get_policy():

    #call slug action server to get policy
    sm_goal = ModeConverterGoal()
    # Sends the goal to the action server.
    sm_cli.send_goal(sm_goal)
    # Waits for the server to finish performing the action.
    sm_cli.wait_for_result(rospy.Duration(2.0))
    sm_result = sm_cli.get_result()
    cmd_state = sm_result.current_mode
    rospy.loginfo("current mode : %d", cmd_state)
    # print "current_context", cmd_state
    return cmd_state

def navigation_action(goal_x,goal_y,goal_yaw):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position = Point(goal_x, goal_y, 0)
    quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
    pose.pose.orientation = Quaternion(*quat)

    goal = MoveBaseGoal()
    goal.target_pose = pose

    # send message to the action server
    cli.send_goal(goal)

    # wait for the action server to complete the order
    cli.wait_for_result()

    # print result of navigation
    result_action_state = cli.get_state()

    return result_action_state 

# def homepose_action():
        # goal = villa_manipulation.msg.HandoverGoal()
        # homepose_client.send_goal(goal)
        # homepose_client.wait_for_result()
        # homeresult_action_state = homepose_client.get_state()
	# return homeresult_action_state

# def receivepose_action():
	# goal = villa_manipulation.msg.HandoverGoal()
	# receivepose_client.send_goal(goal)
	# receivepose_client.wait_for_result()
	# result_action_state = receivepose_client.get_state()
	# return result_action_state 

# def putdown_action():
	# goal = villa_manipulation.msg.ForcePutDownGoal()
	# goal.place_pose =PoseStamped()
	# goal.place_pose.pose.position.x=1.4
	# goal.place_pose.pose.position.y=0.35
	# goal.place_pose.pose.position.z=0.78
	# goal.place_pose.header.frame_id='map'
 
	# put_down_client.send_goal(goal)
	# put_down_client.wait_for_result(rospy.Duration(38.0))
	# rospy.loginfo("38 seconds passed")
	# result_action_state = put_down_client.get_state()
	# actionresult= put_down_client.get_result()
	# print "----putdown_action----"
	# print actionresult

	# if actionresult==GoalStatus.SUCCEEDED:
		# tts.say("putdown succeeded")

	# return result_action_state 
	
def generate_send_goal(cmd_idx, cmd_state, prev_state):

    if cmd_idx ==-1:
            return GoalStatus.SUCCEEDED

    # goal_x = -0.0
    # goal_y = -0.0
    # goal_yaw = 0.0  

    # Move_Base = True

    # cmd_state = desired_states[cmd_idx]
    if cmd_state == 0:
        rospy.loginfo("tracking")
        # move_action_state=navigation_action(goal_x,goal_y,goal_yaw)
    elif cmd_state == 1:
        rospy.loginfo("missing")
    elif cmd_state == 2:
        rospy.loginfo("searching- occ with known object")
    elif cmd_state == 3:
        rospy.loginfo("searching - occ with unknown")
    else:  
        rospy.loginfo("wrong input")

    return GoalStatus.SUCCEEDED


def track_motion_during_duration(counter_in, cmd_state, prev_state):

    cmd_idx=counter_in
    # print "cmd_idx", cmd_idx
    start_time = rospy.get_time()

    # print "start_time", start_time
    duration=0
    iterator=0

    print "cmd_state", cmd_state
    print "prev_state", prev_state

    if cmd_state == 3:
        max_dur = 10.0
    elif cmd_state == prev_state:
        max_dur = 10.0
    else:
        max_dur = 5.0

    print "max_dur", max_dur

    while (duration<max_dur or cmd_idx != -1):

        iterator=iterator+1
        action_state = generate_send_goal(cmd_idx, cmd_state, prev_state)
        curr_time =rospy.get_time()
        # print "curr_time", curr_time
        duration = curr_time - start_time
        # print duration

        # if iterator%10000000==1:
            # print "duration time: %s, action_state %s," % (duration, action_state)
        if action_state == GoalStatus.SUCCEEDED:
            "duration time: %s, action_state %s," % (duration, action_state)
            cmd_idx= -1

        elif action_state == GoalStatus.ACTIVE:
            break
        else:
            break

        iterator=0

        return action_state

class Tracking_State(smach.State):
	def __init__(self):
            smach.State.__init__(self, outcomes=['Go_Tracking', 'Go_Searching','end_demo'],
                    input_keys=['S0_counter_in','S0_counter_out','S0_desired_state_in','S0_desired_state_out','S0_previous_state_in','S0_previous_state_out'],
                    output_keys=['S0_counter_out','S0_desired_state_out','S0_previous_state_out'])

	def execute(self, userdata):
            rospy.loginfo('Executing Tracking_State')

            action_state = track_motion_during_duration(userdata.S0_counter_in, userdata.S0_desired_state_in, userdata.S0_previous_state_in)
            rospy.loginfo("Tracking_state: %d", action_state)

            if action_state == GoalStatus.SUCCEEDED:
                # userdata.S0_counter_out=userdata.S0_counter_in+1
                # print "userdata.S0_counter out", userdata.S0_counter_out
                # userdata.S0_previous_state_out = userdata.S0_desired_state_in
                userdata.S0_desired_state_out, output_state = get_action()
                return output_state

            elif action_state == GoalStatus.Active:
                # userdata.S0_counter_out=userdata.S0_counter_in+1
                # print "userdata.S0_counter out", userdata.S0_counter_out
                # userdata.S0_previous_state_out = userdata.S0_desired_state_in
                userdata.S0_desired_state_out, output_state = get_action()
                return 'Go_Tracking'

            else:
                # "goal was not achieved"
                rospy.loginfo("neither succeeded or active")
                # userdata.S0_counter_out=userdata.S0_counter_in+1
                # print "userdata.S0_counter out", userdata.S0_counter_out
                # userdata.S0_previous_state_out = userdata.S0_desired_state_in
                userdata.S0_desired_state_out, output_state = get_action()
                return 'Go_Searching'


class Searching_State(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['Go_Tracking', 'Go_Searching', 'end_demo'],
                    input_keys=['S1_counter_in','S1_counter_out','S1_desired_state_in','S1_desired_state_out','S1_previous_state_in','S1_previous_state_out'],
                    output_keys=['S1_counter_out','S1_desired_state_out','S1_previous_state_out' ])

        def execute(self, userdata):
            rospy.loginfo('Executing Searching_State')
            # tts.say("Executing State 1")
            # rospy.sleep(0.5)

            action_state = track_motion_during_duration(userdata.S1_counter_in, userdata.S1_desired_state_in, userdata.S1_previous_state_in)

            if action_state == GoalStatus.SUCCEEDED:
                # userdata.S1_counter_out=userdata.S1_counter_in+1
                # print "userdata.S1_counter out", userdata.S1_counter_out
                # userdata.S1_previous_state_out = userdata.S1_desired_state_in
                userdata.S1_desired_state_out, output_state = get_action()
                return output_state
            else:
                "goal was not achieved"
                return 'end_demo'


tts=whole_body = None

while not rospy.is_shutdown():
    try:
        robot = Robot()
        tts = robot.try_get('default_tts')
        whole_body = robot.try_get('whole_body')
        tts.language = tts.ENGLISH
        break
    except (exceptions.ResourceNotFoundError,
            exceptions.RobotConnectionError) as e:
        rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))

limit_moves = 100
# tts.say("Hello! I'm ready'")
rospy.sleep(1)

# initialize action client
cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
sm_cli = actionlib.SimpleActionClient('state_machine', ModeConverterAction)
 
# wait for the action server to establish connection
cli.wait_for_server()
sm_cli.wait_for_server()


if __name__=='__main__':
	# create SMACH state machine
	sm = smach.StateMachine(outcomes=['stop'])
	# sm.userdata.desired_states =desired_states
	sm.userdata.state_index=0
	sm.userdata.current_desired_state=get_policy()
	sm.userdata.previous_desired_state=sm.userdata.current_desired_state

	with sm:
            smach.StateMachine.add('Tracking_State', Tracking_State(),
                    transitions = {'Go_Tracking': 'Tracking_State', 'Go_Searching' : 'Searching_State', 'end_demo' : 'stop'},
                    remapping = {'S0_counter_in':'state_index',
                        'S0_counter_out':'state_index',
                        'S0_desired_state_in' : 'current_desired_state',
                        'S0_desired_state_out' : 'current_desired_state',
                        'S0_previous_state_in' : 'previous_desired_state',
                        'S0_previous_state_out' : 'previous_desired_state'})
            smach.StateMachine.add('Searching_State', Searching_State(),
                            transitions = {'Go_Searching':'Searching_State', 'Go_Tracking':'Tracking_State', 'end_demo':'stop'},
                            remapping = {'S1_counter_in':'state_index',
                                'S1_counter_out':'state_index',
                                'S1_desired_state_in' : 'current_desired_state',
                                'S1_desired_state_out' : 'current_desired_state',
                                'S1_previous_state_in' : 'previous_desired_state',
                                'S1_previous_state_out' : 'previous_desired_state'})

	if sm.userdata.current_desired_state == 0:
		sm.set_initial_state(['Tracking_State'])
	elif sm.userdata.current_desired_state == 1:
		sm.set_initial_state(['Searching_State'])
        else:
            print "initialization wrong"

	
	outcome = sm.execute()


# if __name__ == '__main__':
  # main()
