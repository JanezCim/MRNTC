# REFERENCE PC WORKSPACE

This workspace is ment to be uploaded and compiled on the reference PC. In the REFERENCE PC SETUP LOG section its documented how the software and dependencies on ref PC were setup. In REFERENCE PC RUN there are all the commands that are needed to run all the actions that were pre-prepared.
    

# REFERENCE PC SETUP LOG

## Cartographer

Followed the official tutorial of instalation: https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html
[When compiling the protobuf, the git repo was not accesable from robot (probably because of bad internet connection - it had to be cloned to a PC and then transfered to the robot via sftp)]
All the files neceserry to run cartographer are contained within this workspace, so the only thin needed is to compile it.

## Rtabmap

    sudo apt install ros-kinetic-rtabmap
    sudo apt install ros-kinetic-rtabmap-ros

## Realsense drivers
REALSENSE CAMERA HAS TO BE PLUGGED INTO USB 3.0 PORT OTHERWISE IT CAN START DOING SOME WIERD STUFF

    sudo apt install ros-kinetic-realsense2-camera
    sudo apt install ros-kinetic-realsense2-description

and for the usb permission

    cd
    git clone https://github.com/IntelRealSense/librealsense.git
    cd librealsense/
    ./scripts/setup_udev_rules.sh

## Aruco Detect, fiducials

    sudo apt install ros-kinetic-aruco-detect
    sudo apt install ros-kinetic-fiducials*

## Gazebo for turtlebot (install only if tests in Gazebo are needed)

Gazebo should be already installed with the desktop version of ROS, but for turtlebot:

    sudo apt install ros-kinetic-turtlebot-*

To launch the turtlebot in the mrntc world, use this command

    roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="<ABSOLUTE_PATH_TO_WORKSPACE>/src/mrntc_ref_launch/configuration_files/gazebo_aruco.world"


# REFERENCE PC RUN

## RUNNING SIMULATION OF ARUCO SLAM IN GAZEBO

1. Copy aruco markers (the content of the file Aruco Markers from this repo) and paste them into ~/.gazebo/models

2. Gazebo, turtlebot package, aruco detect and fiductials must be installed

3.      roslaunch mrntc_ref_launch gazebo_turtlebot_aruco_slam.launch

4. Open Rviz and visualise the results

## LAUNCHING RTABMAP 3D MAPPING WITH REALSENSE D435i

    roslaunch mrntc_ref_launch rtabmap.launch


# DEVELOPER NOTES (things I found usefull during development)

1. Whenever using sim time, be sure you start with --clock parameter

        rosbag play *.bag --clock

1. The whole space has to be built with a catkin_make_isolated because cartographer package is not standard

        catkin_make_isolated --install --use-ninja

1. Specific packages inside can be installed using this command

        catkin_make_isolated --install --only-pkg-with-deps PACKAGE_NAME --use-ninja --force    

1. Spawn turtlebot in a gazebo world specified from the commandline

        roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="/home/janez/Documents/Git/ELTE/MRNTC/mrntc_ref_ws/src/mrntc_ref_launch/configuration_files/gazebo_aruco.world"

1. Compile the workspace without cartographer

        catkin_make -DCATKIN_BLACKLIST_PACKAGES="cartographer;cartographer_ros;cartographer_rviz"

