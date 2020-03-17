# REFERENCE PC WORKSPACE

This workspace is ment to be uploaded and compiled on the reference PC
    

# REFERENCE PC SETUP LOG

## Cartographer

Followed the official tutorial of instalation: https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html
[When compiling the protobuf, the git repo was not accesable from robot (probably because of bad internet connection - it had to be cloned to a PC and then transfered to the robot via sftp)]
All the files neceserry to run cartographer are contained within this workspace, so the only thin needed is to compile it.

## Rtabmap

    sudo apt install ros-kinetic-rtabmap

## Aruco Detect, fiducials

    sudo apt install ros-kinetic-aruco-detect
    sudo apt install ros-kinetic-fiducials*

## Gazebo for turtlebot

Gazebo should be already installed with the desktop version of ROS, but for turtlebot:

    sudo apt install ros-kinetic-turtlebot-*

To launch the turtlebot in the mrntc world, use this command

    roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="<ABSOLUTE_PATH_TO_WORKSPACE>/src/mrntc_ref_launch/configuration_files/gazebo_aruco.world"


# RUNNING SIMULATION OF ARUCO SLAM IN GAZEBO

1. Copy aruco markers (the content of the file Aruco Markers from this repo) and paste them into ~/.gazebo/models

2. Gazebo, turtlebot package, aruco detect and fiductials must be installed

3.      roslaunch mrntc_ref_launch gazebo_turtlebot_aruco_slam.launch

4. Open Rviz and visualise the results




# DEVELOPER NOTES

Whenever using sim time, be sure you start with --clock parameter

    rosbag play *.bag --clock

The whole space has to be built with a catkin_make_isolated because cartographer package is not standard

    catkin_make_isolated --install --use-ninja

Specific packages inside can be installed using this command

    catkin_make_isolated --install --only-pkg-with-deps PACKAGE_NAME --use-ninja --force    

Gazebo

    roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="/home/janez/Documents/Git/ELTE/MRNTC/mrntc_ref_ws/src/mrntc_ref_launch/configuration_files/gazebo_aruco.world"
