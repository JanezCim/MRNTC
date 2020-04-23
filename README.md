# Mobile Robot Navigation Techniques Comparison 

This repo is part of an on-going master thesis work. 

**Research question: How do different sensor fusion techniques and localisation algorithms help to provide usable robot localisation with low-cost sensors like cameras, wheel encoders and IMUs?**

Overview of the prosidure:

1. While all the sensors are outputing the data to ros AND cartographer is running within 2D slam mode an Innitial bag file is recorded.

2. The Innitial bag file is filtered to contain only the required TF frames. This step produces the Filtered bag file.

3. Filtered bag file is used to play sensor data that is selectively fused to produce localisation information. This is then compared to the cartographer localisation output and performance evaluation is evaluated.

# 1. Hardware Setup:
2PCs are involved:
 - REF PC: Reference computer that is connected to YDLidar, Realsense d435i
 - TEST PC: PC that is connected to motor drivers and to hokuyo lidar

REF and TEST PC are connected to eachother through LAN connection, where REF PSs network is configured to IPv4 Method: Shared to other computers

TODO picture of the setup.

Both PCs have:
 - ROS: Kinetic
 - Linux: 16.04

# 2. Software setup

Look in each individual workspaces READMEs to setup each PC (TEST and REF)

# 3. Commands needed to collect the Innitial bag file with

TODO collect everything together in a launch file (now not for debugging purposes)

## On REF PC:
New terminal

    cd <REF_WS>
    source ../ros_remote.bash
    source devel/setup.bash
    roscore
    
New terminal for launching realsense d435i

    cd <REF_WS>
    source ../ros_remote.bash
    source devel/setup.bash
    roslaunch mrntc_ref_launch rs_d435i.launch

New terminal for launching ydlidar

    cd <REF_WS>
    source ../ros_remote.bash
    source devel/setup.bash
    roslaunch mrntc_ref_launch ydlidar.launch

New terminal for running the cartographer 

    cd <REF_WS>
    source ../ros_remote.bash
    source devel/setup.bash
    roslaunch mrntc_ref_launch cartographer_live.launch

New terminal for teleop

    cd <REF_WS>
    source ../ros_remote.bash
    source devel/setup.bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

New terminal for recording the bag without the compressed topics

    cd <REF_WS>
    source ../ros_remote.bash
    source devel/setup.bash
    rosbag record -a -x "(.*)/compressed(.*)"

## On TEST PC
New terminal for the driving of motors and odometry

    ydriver

New terminal for running the hokuyo driver

    cd <TEST_WS>
    source ../ros_remote.bash
    source devel/setup.bash
    roslaunch mrntc_test_launch hokuyo.launch serial_port:=/dev/ttyACM0


# TODO: add
## Checking the ATE of cartographer with hokuyo against ground truth:

The bag has to be prepaired in the prosedure described here: (TODO)
The recording of the reference ground trutch check is identical to reference check in all things, but the world frame is set to be the "world_frame" instead of "map" that is because we set the innitial position of the robot with it to corespond to the real orientation of the robot in the beginning.

Inside reference_check.py set the coordinates of the ground truth polygon

    cd <REF_WS>
    source devel/setup.bash
    roslaunch mrntc_ref_launch cartographerhokuyo_odom.launch world_frame:=world_frame bag_path:=<ABSOLUTE_PATH_TO_BAG> log_naming_prefix:=ref10_
    cd src/ate_calculator/scripts/
    python reference_check.py ref10_cartographerhokuyo_odom_coordinates_output.txt
