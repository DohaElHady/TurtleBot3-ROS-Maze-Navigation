#!/usr/bin/env python3
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

home_joints = [0, -1.052, 0.354, 0.703]
# target joints angles using RViz

target_joints_r = [0.0, 0.885 , -0.063 , -0.763]
pre_target_joints_r = [0, 0.58 ,0.54 , -1.055]

# pre_target_joints_r = [0.0, 0.908 , -0.022 , -0.899]
# target_joints_r = [0, 1.008 ,-0.272 , -0.749]

# gripper joints
open_joint = 0.012
close_joint = 0.0

global goal

class MoveGroupInterface(object):
	"""
	MoveGroupInterface Class
	* pick() ===> control manipulator joints angles to pick position
	* place() ===> control manipulator joints angles to drop  position
	* gripper() ===> Gripper function to control prismatic movment of the end effector fingers
	"""
	def __init__(self):
		super(MoveGroupInterface, self).__init__()
  		## `moveit_commander`_ and a `rospy`_ node initialize:
		moveit_commander.roscpp_initialize(sys.argv)

		scene = moveit_commander.PlanningSceneInterface()
  
		group_name = "arm"
		move_group = moveit_commander.MoveGroupCommander(group_name)

		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		moveit_msgs.msg.DisplayTrajectory,
		queue_size=20)

		planning_frame = move_group.get_planning_frame()
		self.box_name = ''
		self.scene = scene
		self.move_group = move_group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame

	def go_to_joint_state(self, joint_goal):
		move_group = self.move_group
		move_group.go(joint_goal, wait=True)
		move_group.stop()
		time.sleep(7)
		return 0


	def gripper(self, gripper_value):
		gripper_publisher = rospy.Publisher('/gripper_position',Float64MultiArray, queue_size =20)
		joint_pos = Float64MultiArray()
		print ("gripper ...... ")
		joint_pos.data = [gripper_value]
		gripper_publisher.publish(joint_pos)
		time.sleep(3)

	def pick(self):
		global home_joints
		global open_joint
		global close_joint
		global i
		print ("Pick 1 is running ..")

		print ("Open gripper")
		self.gripper(open_joint)  # my function
		print ("Go to manipulator target position")
		self.go_to_joint_state(pre_target_joints_r)
		self.go_to_joint_state(target_joints_r)
		time.sleep(3)
		print ("Close gripper")
		self.gripper(close_joint)  # my function
		print ("Go to manipulator home position")
		self.go_to_joint_state(home_joints)

	def place(self):
		global home_joints
		print ("Place function is running ..")

		print ("Go to manipulator place position")
		self.go_to_joint_state(target_joints_r)

		print ("Open gripper")
		self.gripper(open_joint)

		self.go_to_joint_state(pre_target_joints_r)

		print ("Go to manipulator home position")
		self.go_to_joint_state(home_joints)

		print ("Close gripper")
		self.gripper(close_joint)


# Callbacks definition
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

def moveToGoal(list_poses):
	global goal
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	set_pos(list_poses)
	print(goal.target_pose.pose.position.x)
	print(goal.target_pose.pose.position.y)
	print(goal.target_pose.pose.position.z)
	navigation_client.send_goal(goal, done_cb, active_cb, feedback_cb)
	finished = navigation_client.wait_for_result()
	if not finished:
	    rospy.logerr("Action server not available!")
	else:
	    rospy.loginfo(navigation_client.get_result())


def exitHome():
	# moveToGoal([2.44,0.85,0,0,0,0,1])
	# moveToGoal([2.62,0.85,0,0,0,0,1])
	# moveToGoal([2.65,0.85,0,0,0,-0.33,0.94])
	# moveToGoal([3.28,0.40,0,0,0,-0.33,0.94])#out of home
	moveToGoal([2.65,0.85,0,0,0,0,1])
	moveToGoal([2.65,0.85,0,0,0,-0.33,0.94])
	moveToGoal([3.28,0.45,0,0,0,-0.33,0.94])#out of home

def goToTarget1():
	# moveToGoal([3.28,0.40,0,0,0,0,1])
	# moveToGoal([3.9,0.40,0,0,0,0,1])
	# moveToGoal([3.9,0.40,0,0,0,0.7,0.71])
	# moveToGoal([3.9,2,0,0,0,0.7,0.71])
	# moveToGoal([3.9,3.28,0,0,0,0.7,0.71])
	# moveToGoal([3.9,3.28,0,0,0,0.7,0.71])#Target 1

	moveToGoal([3.28,0.45,0,0,0,0,1])
	moveToGoal([3.85,0.45,0,0,0,0,1])
	moveToGoal([3.85,0.45,0,0,0,0.7,0.71])
	moveToGoal([3.85,2,0,0,0,0.7,0.71])
	moveToGoal([3.85,3.30,0,0,0,0.7,0.71])#Target 1




def goToPlace1():
	moveToGoal([3.85,3.30,0,0,0,0.7,-0.70])
	moveToGoal([3.85,2.00,0,0,0,0.7,-0.70])
	moveToGoal([3.9,0.45,0,0,0,0.7,-0.71])
	#moveToGoal([2.5,0.22,0,0,0,1,0])
	moveToGoal([1.58,0.3,0,0,0,1,0])
	moveToGoal([1.58,1.48,0,0,0,0.7,0.71])
	moveToGoal([1.58,1.48,0,0,0,1,0])
	moveToGoal([0.90,1.48,0,0,0,1,0])
	moveToGoal([0.90,1.48,0,0,0,0.7,0.7])
	moveToGoal([0.90,2.25,0,0,0,0.7,0.7])
	moveToGoal([0.96,3.52,0,0,0,0.7,0.7])
	moveToGoal([0.96,3.52,0,0,0,0,1])
	moveToGoal([1.80,3.56,0,0,0,0,1])#Place 1


def goToTarget2():
	moveToGoal([1.60,3.52,0,0,0,0,1])#move back to not kick the object
	moveToGoal([1.60,3.52,0,0,0,1,0])
	moveToGoal([0.96,3.52,0,0,0,1,0])
	moveToGoal([0.96,3.55,0,0,0,0.7,-0.7])
	moveToGoal([0.90,2.25,0,0,0,0.7,-0.7])
	moveToGoal([0.90,1.48,0,0,0,0.7,-0.7])
	moveToGoal([0.90,1.48,0,0,0,0,1])
	moveToGoal([2.44,1.48,0,0,0,0,1])#Target 2


def goToPlace2():
	moveToGoal([2.44,1.48,0,0,0,1,0])
	moveToGoal([0.90,1.48,0,0,0,1,0])
	moveToGoal([0.90,1.48,0,0,0,0.7,0.7])
	moveToGoal([0.90,2.25,0,0,0,0.7,0.7])
	moveToGoal([0.96,3.52,0,0,0,0.7,0.7])#Place 2


def backToHome():
	moveToGoal([0.96,3.35,0,0,0,0.7,0.7])#move back to not kick the object
	moveToGoal([0.96,3.35,0,0,0,0.7,-0.7])
	moveToGoal([0.90,2.25,0,0,0,0.7,-0.7])
	moveToGoal([0.90,1.48,0,0,0,0.7,-0.7])
	moveToGoal([0.90,1.48,0,0,0,0,1])#rotate
	moveToGoal([1.58,1.48,0,0,0,0,1])
	moveToGoal([1.58,1.48,0,0,0,0.7,-0.71])
	moveToGoal([1.58,0.30,0,0,0,0.7,-0.71])
	moveToGoal([1.58,0.30,0,0,0,0,1])#rotate
	moveToGoal([3.28,0.40,0,0,0,0,1])
	moveToGoal([3.28,0.40,0,0,0,-0.94,-0.33])
	moveToGoal([2.66,0.85,0,0,0,-0.94,-0.33])
	moveToGoal([2.44,0.85,0,0,0,1,0])


rospy.init_node('goal_pose', anonymous=True)

navigation_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
navigation_client.wait_for_server()

## manipulator object 
manipulator = MoveGroupInterface()
manipulator.gripper(0)  #Fake Open

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

