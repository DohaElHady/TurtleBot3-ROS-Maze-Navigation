#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# Node initialization
rospy.init_node('init_pose')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 1)

# Construct message
pose_msg = PoseWithCovarianceStamped()
pose_msg.header.frame_id = "map"

# Get initial pose from Gazebo
# wait for nav_msgs/Odometry message from the odom topic
 
# odom_msg = rospy.wait_for_message('/odom', Odometry)
# pose_msg.pose.pose.position.x = odom_msg.pose.pose.position.x
# pose_msg.pose.pose.position.y = odom_msg.pose.pose.position.y
# pose_msg.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
# pose_msg.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
# pose_msg.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
# pose_msg.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

# # for real environment
# pose_msg.pose.pose.position.x = 2.42
# pose_msg.pose.pose.position.y = 0.89
pose_msg.pose.pose.position.x = 2.44
pose_msg.pose.pose.position.y = 0.85
pose_msg.pose.pose.orientation.x = 0
pose_msg.pose.pose.orientation.y = 0
# pose_msg.pose.pose.orientation.z = 0.04
pose_msg.pose.pose.orientation.z = 0.0

pose_msg.pose.pose.orientation.w = 1
# # Delay
rospy.sleep(1)

# Publish message with initial positions
rospy.loginfo("setting initial pose")
pub.publish(pose_msg)
rospy.loginfo("initial pose is set")
