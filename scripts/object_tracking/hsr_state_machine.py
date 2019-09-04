#!/usr/bin/env python

import actionlib
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from deepgaze_ros.msg import ModeConverterAction, ModeConverterGoal
from villa_manipulation.msg import *
from navi_service.msg import *
from std_msgs.msg import Int32MultiArray
from tmc_msgs.msg import BatteryState
from hsrb_interface import Robot
from hsrb_interface import exceptions
from visual_perception.msg import *
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



def get_action(current_state):

        # if cmd_idx==len(desired_states):
    current_context=get_policy()

    if current_state==0:
        if current_context == 0:
            output_state = 'Go_Tracking'
        elif current_context == -1:
            if current_state == 0:
                output_state = 'Go_Tracking'
            else:
                output_state = 'Go_Searching'
        else:
            output_state = 'Go_Searching'

        return current_context, output_state
    else:
        if current_context == 0:
            output_state = 'Go_Searching'
        elif current_context == -1:
            if current_state == 0:
                output_state = 'Go_Searching'
            else:
                output_state = 'Go_Searching'
        else:
            output_state = 'Go_Searching'

        return current_context, output_state
    # elif current_context == 1:
        # output_state = 'Go_Searching'
    # else:  
        # print "desired state out of bounds"
        # output_state = 'end_demo'
    # return current_context, output_state

def sm_feedback_Cb(msg):
    global is_feedback
    global opt_pose
    global last_target_pos
    global is_obstacle
    rospy.loginfo("feedback callback")
    is_feedback=True
    opt_pose =msg.opt_pose
    last_target_pos =msg.last_target_position
    is_obstacle = msg.is_obstacle
    # print msg

def get_policy():
    #call slug action server to get policy
    sm_goal = ModeConverterGoal()
    # Sends the goal to the action server.
    sm_cli.send_goal(sm_goal, done_cb=None,active_cb=None,feedback_cb=sm_feedback_Cb)
    # Waits for the server to finish performing the action.
    # sm_cli.wait_for_result(rospy.Duration(2.0))
    sm_cli.wait_for_result()
    sm_result = sm_cli.get_result()
    cmd_state = sm_result.current_mode
    rospy.loginfo("current mode : %d", cmd_state)
    # print "current_context", cmd_state
    return cmd_state

def headtracking_action():
    goal = villa_manipulation.msg.HeadTrackingGoal()
    headtracking_cli.send_goal(goal)
    headtracking_cli.wait_for_result()
    rospy.loginfo("head_tracking action completed")
    headtracking_actionresult = headtracking_cli.get_state()
    return headtracking_actionresult

# def handtracking_action():
    # goal = villa_manipulation.msg.HandoverGoal()
    # headtracking_cli.send_goal(goal)
    # headtracking_cli.wait_for_result()
    # rospy.loginfo("head_tracking action completed")
    # headtracking_actionresult = headtracking_cli.get_state()
    # return headtracking_actionresult


def search_action():
    goal = visual_perception.msg.SearchGoal()
    search_cli.send_goal(goal)
    search_cli.wait_for_result()
    rospy.loginfo("search_action completed")
    search_result = search_cli.get_state()
    return search_result


def approach_action(desired_pose):
    # rospy.loginfo("approach_action_")
    global is_feedback
    global opt_pose
    global last_target_pos
    global is_obstacle

    print "is_feedback", is_feedback
    print "is_obstacle", is_obstacle
    if is_feedback:

        # rospy.loginfo("approach_action if inside")
        goal = navi_service.msg.ApproachGoal()

        # desired_pose=PoseStamped()
        desired_pose.header.frame_id='map'
        desired_pose.header.stamp=rospy.Time.now()
        goal.target=desired_pose
        # rospy.loginfo("approac_action middle")
        print('----------------------------------------------------------------')
        print desired_pose
        print('----------------------------------------------------------------')
        gaze_pose = geometry_msgs.msg.PoseStamped()
        gaze_pose.header.frame_id='map'
        gaze_pose.header.stamp=rospy.Time.now()
        gaze_pose.pose.position.x=last_target_pos.x
        gaze_pose.pose.position.y=last_target_pos.y
        if is_obstacle==False:
            goal.gaze_target = gaze_pose
            approach_client.send_goal(goal)
            approach_client.wait_for_result()
            approach_state = approach_client.get_state()
            # rospy.loginfo("approach action state = ", approach_state)
            return approach_state 
        else:
            rospy.loginfo("approach cannot be called because of obstacles")
            return 3
        # return 3
    else:
        rospy.loginfo("approach_else")
        return 3




def navigation_action(desired_pose):
    # pose = PoseStamped()
    desired_pose.header.stamp = rospy.Time.now()
    desired_pose.header.frame_id = "map"
    # quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
    # pose.pose.orientation = Quaternion(*quat)
    goal = MoveBaseGoal()
    goal.target_pose = desired_pose

    # send message to the action server
    cli.send_goal(goal)

    # wait for the action server to complete the order
    cli.wait_for_result()

    # print result of navigation
    result_action_state = cli.get_state()

    return result_action_state 

def moveit_ik_action():
    goal = villa_manipulation.msg.Move_IkGoal()
    goal.pose = PoseStamped()
    p = geometry_msgs.msg.PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose = opt_pose
    # goal.pose.pose.position.z = 0.8
    goal.pose.pose.orientation.x = -0.5
    goal.pose.pose.orientation.y= 0.5
    goal.pose.pose.orientation.z = -0.5
    goal.pose.pose.orientation.w = 0.5
    goal.desired_head_pan =0.0
    move_ik_client.send_goal(goal)
    move_ik_client.wait_for_result()
 
    moveit_ik_action_state = move_ik_client.get_state()
    return moveit_ik_action_state 

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
    global opt_pose

    # rospy.loginfo("generate_send_goal, cmd_state: %d", cmd_state)
    # if cmd_idx equals to -1, action server is running 
    if cmd_idx ==-1:
            return GoalStatus.SUCCEEDED

    # Move_Base = True
    action_state=GoalStatus.SUCCEEDED
    # cmd_state = desired_states[cmd_idx]
    if cmd_state == 0:
        rospy.loginfo("tracking with head&base")
        approach_client.cancel_all_goals()
        search_cli.cancel_all_goals()
        action_state = headtracking_action()
        # move_action_state=navigation_action(goal_x,goal_y,goal_yaw)
    elif cmd_state == 1:
        rospy.loginfo("missing")
        rospy.loginfo("tracking person")
        rospy.loginfo("generate_send_goal, cmd_state: %d", cmd_state)
        action_state = search_action()
        # action_state = headtracking_action()
    elif cmd_state == 2:
        # action_state = ee_control_action()
        rospy.sleep(0.5)
        rospy.loginfo("searching- occ with known object")
        # action_state = moveit_ik_action()
        action_state = approach_action(opt_pose)
    elif cmd_state == 3:
        # action_state = headtracking_action()
        rospy.sleep(0.5)
        action_state = approach_action(opt_pose)
        # action_state=navigation_action(opt_pose)
        # action_state = moveit_ik_action()
        rospy.loginfo("searching - occ with unknown")

    else:  
        rospy.loginfo("wrong input/current cmd state is %d", cmd_state)

    return action_state


def track_motion_during_duration( cmd_state, prev_state):
    cmd_idx=0
    # print "cmd_idx", cmd_idx
    start_time = rospy.get_time()

    # print "start_time", start_time
    duration=0
    iterator=0

    # print "cmd_state", cmd_state
    # print "prev_state", prev_state

    if cmd_state == 3:
        max_dur = 5.0
    elif cmd_state ==2:
        max_dur = 5.0
    elif cmd_state ==1:
        max_dur = 2.5
    else:
        max_dur = 0.5

    # print "max_dur", max_dur
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
            rospy.loginfo("action is still active")
            continue
        else:
            # rospy.loginfo(")
            rospy.loginfo("output is wrong, current action_state is : %d",action_state  )
            continue

        iterator=0

    return action_state

class Tracking_State(smach.State):
	def __init__(self):
            smach.State.__init__(self, outcomes=['Go_Tracking', 'Go_Searching','end_demo'],
                    input_keys=['S0_counter_in','S0_counter_out','S0_desired_state_in','S0_desired_state_out','S0_previous_state_in','S0_previous_state_out'],
                    output_keys=['S0_counter_out','S0_desired_state_out','S0_previous_state_out'])

	def execute(self, userdata):
            rospy.loginfo('Executing Tracking_State')

            action_state = track_motion_during_duration(userdata.S0_desired_state_in, userdata.S0_previous_state_in)
            # rospy.loginfo("Tracking_state: %d", action_state)

            if action_state == GoalStatus.SUCCEEDED:
                # userdata.S0_counter_out=userdata.S0_counter_in+1
                # print "userdata.S0_counter out", userdata.S0_counter_out
                userdata.S0_previous_state_out = userdata.S0_desired_state_in
                userdata.S0_desired_state_out, output_state = get_action(0)
                return output_state

            elif action_state == GoalStatus.Active:
                # userdata.S0_counter_out=userdata.S0_counter_in+1
                # print "userdata.S0_counter out", userdata.S0_counter_out
                userdata.S0_previous_state_out = userdata.S0_desired_state_in
                userdata.S0_desired_state_out, output_state = get_action(0)
                return 'Go_Tracking'

            else:
                # "goal was not achieved"
                rospy.loginfo("neither succeeded or active")
                # userdata.S0_counter_out=userdata.S0_counter_in+1
                # print "userdata.S0_counter out", userdata.S0_counter_out
                userdata.S0_previous_state_out = userdata.S0_desired_state_in
                userdata.S0_desired_state_out, output_state = get_action(0)
                return 'Go_Searching'


class Searching_State(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['Go_Tracking', 'Go_Searching', 'end_demo'],
                    input_keys=['S1_counter_in','S1_counter_out','S1_desired_state_in','S1_desired_state_out','S1_previous_state_in','S1_previous_state_out'],
                    output_keys=['S1_counter_out','S1_desired_state_out','S1_previous_state_out' ])

        def execute(self, userdata):
            rospy.loginfo('Executing Searching_State')

            action_state = track_motion_during_duration(userdata.S1_desired_state_in, userdata.S1_previous_state_in)

            if action_state == GoalStatus.SUCCEEDED:
                # userdata.S1_counter_out=userdata.S1_counter_in+1
                # print "userdata.S1_counter out", userdata.S1_counter_out
                userdata.S1_previous_state_out = userdata.S1_desired_state_in
                userdata.S1_desired_state_out, output_state = get_action(1)
                return output_state
            else:
                # "goal was not achieved"
                rospy.loginfo("neither succeeded or active")
                # userdata.S0_counter_out=userdata.S0_counter_in+1
                # print "userdata.S0_counter out", userdata.S0_counter_out
                userdata.S1_previous_state_out = userdata.S1_desired_state_in
                userdata.S1_desired_state_out, output_state = get_action(1)
                return 'Go_Searching'


            # else:
                # "goal was not achieved"
                # return 'end_demo'


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

opt_pose = PoseStamped()
gaze_pose = PoseStamped()
last_target_pos = Point()
is_feedback=False

rospy.sleep(1)
# initialize action client
cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
sm_cli = actionlib.SimpleActionClient('state_machine', ModeConverterAction)
headtracking_cli = actionlib.SimpleActionClient('headtracking_action', villa_manipulation.msg.HeadTrackingAction)
search_cli = actionlib.SimpleActionClient('search_server', visual_perception.msg.SearchAction)
# move_ik_client = actionlib.SimpleActionClient('moveit_ik',villa_manipulation.msg.Move_IkAction)
approach_client = actionlib.SimpleActionClient('approach_action',navi_service.msg.ApproachAction)
# wait for the action server to establish connection
cli.wait_for_server()
search_cli.wait_for_server()
sm_cli.wait_for_server()
headtracking_cli.wait_for_server()
# move_ik_client.wait_for_server()
approach_client.wait_for_server()

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
