#!/usr/bin/env python3

# roscore # on master before running bringup

#software
# roslaunch turtle_7bhan project_1.launch
# roslaunch turtle_7bhan project_navigation.launch

#hardware
# roslaunch turtle_7bhan navigation_manipulation.launch

#on turtle
#roslaunch turtlebot3_bringup turtlebot3_robot.launch

# on both hardware and software
# rosrun turtle_7bhan init_pose.py
# rosrun turtle_7bhan project_poses.py

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import open_manipulator_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from open_manipulator_msgs.srv import *
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
import trajectory_msgs.msg as tm

home_joints = [0, -1.052, 0.354, 0.703]
target1_joints = [0.0, 1.148 , -0.24 , -0.418]
target2_joints = [-1.578, 1.148 , -0.24 , -0.418]


open_joint = 0.01#0.07
close_joint = -0.01 #-0.01
i = 0

global goal

class MoveGroupInterface(object):
	"""MoveGroupInterface"""
	#global i
	def __init__(self):
		super(MoveGroupInterface, self).__init__()
		## `moveit_commander`_ and a `rospy`_ node initialize:
		moveit_commander.roscpp_initialize(sys.argv)

		robot = moveit_commander.RobotCommander()

		scene = moveit_commander.PlanningSceneInterface()

		group_name = "arm"
		move_group = moveit_commander.MoveGroupCommander(group_name)
		gripper_group = moveit_commander.MoveGroupCommander("gripper")
		gripper_group_variable_values = gripper_group.get_current_joint_values()
		
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		moveit_msgs.msg.DisplayTrajectory,
		queue_size=20)

		planning_frame = move_group.get_planning_frame()

		eef_link = move_group.get_end_effector_link()

		group_names = robot.get_group_names()

		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.move_group = move_group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names
		self.gripper_group=gripper_group
		self.gripper_group_variable_values=gripper_group_variable_values
		
	def go_to_joint_state(self, joint_goal):
		move_group = self.move_group
		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		move_group.go(joint_goal, wait=True)
		# Calling ``stop()`` ensures that there is no residual movement
		move_group.stop()
		time.sleep(7)
		return 0


		
	def open_gripper(self):
		print ("Opening Gripper...")
		self.gripper_group_variable_values[0] = 00.009
		self.gripper_group.set_joint_value_target(self.gripper_group_variable_values)
		plan2 = self.gripper_group.go()
		self.gripper_group.stop()
		self.gripper_group.clear_pose_targets()
		rospy.sleep(1)

	def close_gripper(self):
		print ("Closing Gripper...")
		self.gripper_group_variable_values[0] = -00.0006
		self.gripper_group.set_joint_value_target(self.gripper_group_variable_values)
		plan2 = self.gripper_group.go()
		self.gripper_group.stop()
		self.gripper_group.clear_pose_targets()
		rospy.sleep(1)

	def pick(self):
		global home_joints
		global target1_joints
		global open_joint
		global close_joint
		global i
		print ("Pick 1 is running ..")

		if (i == 0):
			print (i)
			print ("Go to home position")
			## Comment next line when using the real robot
			self.go_to_joint_state(home_joints)

		
		print ("Open gripper")
		self.open_gripper()
		print ("Go to target 1")
		self.go_to_joint_state(target1_joints)
		print ("Close gripper")
		self.close_gripper()
		
		print ("Go to home position")
		self.go_to_joint_state(home_joints)
		i=1

	def place(self):
		global home_joints
		global target2_joints
		print ("Place 1 is running ..")

		print ("Go to target 2")
		self.go_to_joint_state(target1_joints)

		print ("Open gripper")
		self.open_gripper()
		print ("Go to home position")
		self.go_to_joint_state(home_joints)
		print ("Close gripper")
		self.close_gripper()
	

def active_cb(extra=None):
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    rospy.loginfo("Current location: "+str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")


def set_pos(list_poses):
	global goal
	[goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,goal.target_pose.pose.position.z,
	goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y,goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w]=list_poses

def move_to_goal(list_poses):
	global goal
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	set_pos(list_poses)
	print(goal.target_pose.pose.position.x)
	navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
	finished = navclient.wait_for_result()


	if not finished:
	    rospy.logerr("Action server not available!")
	else:
	    rospy.loginfo ( navclient.get_result())




def exitHome():
	move_to_goal([2.7,0.95,0,0,0,0,1])
	move_to_goal([2.67,0.95,0,0,0,-0.33,0.94])
	move_to_goal([3.28,0.47,0,0,0,-0.33,0.94])

def goToTarget1():
	move_to_goal([3.28,0.47,0,0,0,0,1])
	move_to_goal([3.93,0.5,0,0,0,0,1])
	move_to_goal([3.93,0.5,0,0,0,0.7,0.71])
	move_to_goal([3.98,2.35,0,0,0,0.7,0.71])
	move_to_goal([3.93,3.25,0,0,0,0.7,0.71])
	move_to_goal([3.93,3.10,0,0,0,0.7,0.71])



def goToPlace1():
	move_to_goal([3.93,3.10,0,0,0,0.7,-0.70])
	move_to_goal([3.98,2.35,0,0,0,0.7,-0.70])
	move_to_goal([3.98,0.5,0,0,0,0.7,-0.71])
	move_to_goal([1.58,0.3,0,0,0,1,0])
	move_to_goal([1.58,1.58,0,0,0,0.7,0.71])
	move_to_goal([1.58,1.58,0,0,0,1,0])
	move_to_goal([0.98,1.58,0,0,0,1,0])
	move_to_goal([0.98,1.58,0,0,0,0.7,0.7])
	move_to_goal([0.98,2,0,0,0,0.7,0.7])
	move_to_goal([1,3.59,0,0,0,0.7,0.7])
	move_to_goal([1,3.59,0,0,0,0,1])
	move_to_goal([1.76,3.59,0,0,0,0,1])#Place 1



def goToTarget2():
	move_to_goal([1.61,3.59,0,0,0,0,1])#move back to not kick the object
	move_to_goal([1.61,3.59,0,0,0,1,0])
	move_to_goal([1,3.59,0,0,0,1,0])
	move_to_goal([1,3.59,0,0,0,0.7,-0.7])
	move_to_goal([0.98,2,0,0,0,0.7,-0.7])
	move_to_goal([0.98,1.58,0,0,0,0.7,-0.7])
	move_to_goal([0.98,1.58,0,0,0,0,1])
	move_to_goal([2.25,1.55,0,0,0,0,1])#Target 2


def goToPlace2():
	move_to_goal([2.10,1.58,0,0,0,0,1])
	move_to_goal([2.10,1.58,0,0,0,1,0])
	move_to_goal([0.98,1.58,0,0,0,1,0])
	move_to_goal([0.98,1.58,0,0,0,0.7,0.7])
	move_to_goal([0.98,2,0,0,0,0.7,0.7])
	move_to_goal([1,3.4,0,0,0,0.7,0.7])#Place 2


def backToHome():
	move_to_goal([1,3.35,0,0,0,0.7,0.7])#move back to not kick the object
	move_to_goal([1,3.35,0,0,0,0.7,-0.7])
	move_to_goal([0.98,2,0,0,0,0.7,-0.7])
	move_to_goal([0.98,1.58,0,0,0,0.7,-0.7])
	move_to_goal([0.98,1.58,0,0,0,0,1])#rotate
	move_to_goal([1.58,1.58,0,0,0,0,1])
	move_to_goal([1.58,1.58,0,0,0,0.7,-0.71])
	move_to_goal([1.58,0.35,0,0,0,0.7,-0.71])
	move_to_goal([1.58,0.35,0,0,0,0,1])#rotate
	move_to_goal([3.28,0.47,0,0,0,0,1])
	move_to_goal([3.28,0.47,0,0,0,-0.94,-0.33])
	move_to_goal([2.66,0.95,0,0,0,-0.94,-0.33])
	move_to_goal([2.4,0.95,0,0,0,1,0])



rospy.init_node('goal_pose')
manipulator = MoveGroupInterface()
navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
navclient.wait_for_server()

# exit home and go to the 1st pick position
exitHome() 
goToTarget1()
manipulator.pick()

# go to 1st drop position to place object
goToPlace1()
manipulator.place()

# go to 2nd target and pick object
goToTarget2()
manipulator.pick()

# go to 2nd drop position to place object
goToPlace2()
manipulator.place()

# return back to home
backToHome()

