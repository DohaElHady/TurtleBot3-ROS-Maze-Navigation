************* turtle_7bhan Package *************
*************      ROS neotic      *************
************************************************
Dependancies :
	cpp_common
	rostime
	roscpp_traits
	roscpp_serialization
	catkin
	genmsg
	genpy
	message_runtime
	gencpp
	geneus
	gennodejs
	genlisp
	message_generation
	rosbuild
	rosconsole
	std_msgs
	rosgraph_msgs
	xmlrpcpp
	roscpp
	rosgraph
	ros_environment
	rospack
	roslib
	rospy
************************************************
Installation Steps:
- Download the attached turtle_7bhan.zip
- Unzip the folder to your catkin_ws
- Run the following commands:
	$ cd ~/catkin_ws
	$ rosdep install --from-paths src --ignore-src -r -y
	$ catkin_make
- Open project_map.yaml and change the $user to your PC username
************************************************
Run Simulation:
$ roslaunch turtle_7bhan turtlebot3_manipulation_gazebo_project1.launch
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch	
#### then click play button in gazebo
$ roslaunch turtle_7bhan navigation_manipulation_project1.launch
$ rosrun turtle_7bhan project_poses_with_pick.py
************************************************
Run Simulation without Manipulator:
$ roslaunch turtle_7bhan turtlebot3_gazebo_project1.launch
$ roslaunch turtle_7bhan navigation_project1.launch
$ rosrun turtle_7bhan project_poses.py
************************************************
Run on Real Map:
$ roslaunch turtle_7bhan 
$ turtlebot3_manipulation_gazebo_project1.launch
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch	
#### then click play button in gazebo
$ roslaunch turtle_7bhan navigation_manipulation_project1.launch
$ rosrun turtle_7bhan init_pose.py
$ rosrun turtle_7bhan project_poses_with_pick.py	
	
	
	
	
	
	
	
	
	

