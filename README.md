# ArUco visual control with ROS2

Final project for TASFR.

This project implements a simple visual servoing system using ArUco markers in ROS2 Humble.

## Description
- A camera publishes images on `/image`
- An ArUco marker is detected
- The robot moves forward/backward depending on the vertical position of the marker
- Motion commands are published on `/cmd_vel`
- A simple TF is generated from cmd_vel to visualize motion in RViz

## Packages
- `aruco_final`
	- `aruco_contro_node.py`: detects ArUco markers and generates cmd_vel
	- `cmdvel_to_tf.py` : converts cmd_vel into TF/odometry for RViz visualization

## How to run


Terminal 1 (camera):
source $HOME/ros2_ws/install/setup.bash;
ros2 run image_tools cam2image -p device_id:=0

Terminal 2 (ArUco control):
source $HOME/ros2_ws/install/setup.bash;
ros2 run aruco_final aruco_control_node

Terminal 3 (cmd_vel to TF):
source $HOME/ros2_ws/install/setup.bash;
ros2 run aruco_final cmdvel_to_tf

Terminal 4 (RViz):
source $HOME/ros2_ws/install/setup.bash;
rviz2

In RViz:
- Fixed frame: odom
- Add TF, Odometry (topic: `/odom`, Image (topic: `/aruco/image_annotated`), RobotModel (description source: topic, description topic: `/robot_description`)

NOTES
This project uses RViz only for visualization. No simulator is used.
