# REFERENCE PC WORKSPACE

This workspace is ment to be uploaded and compiled on the reference PC. In the REFERENCE PC SETUP LOG section its documented how the software and dependencies on ref PC were setup. In REFERENCE PC RUN there are all the commands that are needed to run all the actions that were pre-prepared.
    

# REFERENCE PC SETUP LOG

## Cartographer

    Followed the official tutorial of instalation: https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html
    [When compiling the protobuf, the git repo was not accesable from robot (probably because of bad internet connection - it had to be cloned to a PC and then transfered to the robot via sftp)]
    All the files neceserry to run cartographer are contained within this workspace, so the only thin needed is to compile it.

## Realsense driver
REALSENSE CAMERA HAS TO BE PLUGGED INTO USB 3.0 PORT OTHERWISE IT CAN START DOING SOME WIERD STUFF

1. FROM SOURCE (prefered and used in this project):

    Followed official tutorial on https://github.com/IntelRealSense/realsense-ros
    (17.7.2019) \ Istalling librealsense (Step 1 on the Realsense Github), HAS TO BE DONE from source and not from Debian package! When following instructions how to build from source (https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md), checkout to development branch before building (Building librealsense2 SDK Step). Why is that is described here (https://github.com/IntelRealSense/realsense-ros/issues/703#issuecomment-483666317)


2. INSTALLING WITH APT:

        sudo apt install ros-kinetic-realsense2-camera
        sudo apt install ros-kinetic-realsense2-description

    and for the usb permission

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

## YDlidar

Followed official tutorial https://github.com/EAIBOT/ydlidar

# DEVELOPER NOTES (things I found usefull during development)

## 1. RUNNING SIMULATION OF ARUCO SLAM IN GAZEBO

1. Copy aruco markers (the content of the file Aruco Markers from this repo) and paste them into ~/.gazebo/models

2. Gazebo, turtlebot package, aruco detect and fiductials must be installed

3.      roslaunch mrntc_ref_launch gazebo_turtlebot_aruco_slam.launch

4. Open Rviz and visualise the results

## 2. LAUNCHING RTABMAP 3D MAPPING WITH REALSENSE D435i

        roslaunch mrntc_ref_launch rtabmap.launch

Whenever using sim time, be sure you start with --clock parameter

        rosbag play *.bag --clock

## 3. General notes

1. The whole space has to be built with a catkin_make_isolated because cartographer package is not standard

        catkin_make_isolated --install --use-ninja

1. Specific packages inside can be installed using this command

        catkin_make_isolated --install --only-pkg-with-deps PACKAGE_NAME --use-ninja --force    

1. Spawn turtlebot in a gazebo world specified from the commandline

        roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="/home/janez/Documents/Git/ELTE/MRNTC/mrntc_ref_ws/src/mrntc_ref_launch/configuration_files/gazebo_aruco.world"

1. Compile the workspace without cartographer

        catkin_make -DCATKIN_BLACKLIST_PACKAGES="cartographer;cartographer_ros;cartographer_rviz"

1. Save the map from cartographer with map server for the use in amcl package (TODO add in appropriate readme)

        cd src/mrntc_ref_launch/maps/
        rosrun map_server map_saver -f room_ydlidar --occ 65 --free 20

1. Prosedure to record a bag with realsense d435i and then making a 3d map and visual odometry with rtabmap from that bag
        
        Worked for me by launching the realsense launch [rs_camera.launch](https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/launch/rs_camera.launch) with following changes: 

        <arg name="enable_fisheye"      default="false"/> <!--disabled for less data traffic -->
        <arg name="align_depth"               default="false"/>
        <arg name="unite_imu_method"          default="linear_interpolation"/>
          <arg name="publish_odom_tf"           default="false"/>

        and by recording the bag with out compressed or theora: 

        rosbag record -a -x "(.*)/compressed(.*)|(.*)/theora(.*)"

        After I recorded the bag with this command and launched the rtabmap with default [rtabmap.launch](https://github.com/introlab/rtabmap_ros/blob/master/launch/rtabmap.launch), the 3d mapping and visual odometry worked.
