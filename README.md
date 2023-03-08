# TurtleBot3 ROS-based Maze Navigation
This repository includes a ROS package for TurtleBot3 maze navigation project

## Project Requirements 
Autonomous navigation through lanes built of red tapes and walls, with the objective of reaching picking and placing locations from the robot’s home position by avoiding obstacles and manipulating objects, and finally returning back of the robot to its home.

## Package Description
This package deploys the navigation package for SLAM and autonomous navigation with obstacle avoidance. 
It deploys viapoint approach for lane following without crossing the red tapes.
MoveIt package was used for manipulator joint angles control and gripper control, for a pick and place task. 


# turtle_7bhan Package (ROS neotic)

## Installation Steps:
- Download the attached turtle_7bhan.zip
- Unzip the folder to your catkin_ws
- Run the following commands:

	$ cd ~/catkin_ws
  
	$ rosdep install --from-paths src --ignore-src -r -y
  
	$ catkin_make
  
- Open project_map.yaml and change the $user to your PC username
************************************************
## Run Gazebo Simulation:

$ roslaunch turtle_7bhan turtlebot3_manipulation_gazebo_project1.launch

$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch	

- Then click play button in gazebo
$ roslaunch turtle_7bhan navigation_manipulation_project1.launch

$ rosrun turtle_7bhan project_poses_with_pick.py
************************************************
## Run Gazebo Simulation without Manipulator:

$ roslaunch turtle_7bhan turtlebot3_gazebo_project1.launch

$ roslaunch turtle_7bhan navigation_project1.launch

$ rosrun turtle_7bhan project_poses.py
************************************************
## Contributors
[Doha ElHady](https://github.com/DohaElHady) - [Mohamed Elahl](https://github.com/MohamedElahl) - [Hassan Mohamed](https://github.com/Hsnmhmd) - [Karim Youssef](https://github.com/KarimYoussef98)
