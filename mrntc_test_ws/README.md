# TEST PC WORKSPACE

This workspace is ment to be uploaded and compiled on the PC that the sensors will be tested on


# Building 

    catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
    catkin_make install


# Test PC preparation commands

HOKUYU DRIVER INSTALL

    sudo apt install ros-kinetic-urg-*

REALSENSE T265

Followed official tutorial on https://github.com/IntelRealSense/realsense-ros
(17.7.2019) \ Istalling librealsense (Step 1 on the Realsense Github), HAS TO BE DONE from source and not from Debian package! When following instructions how to build from source (https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md), checkout to development branch before building (Building librealsense2 SDK Step). Why is that is described here (https://github.com/IntelRealSense/realsense-ros/issues/703#issuecomment-483666317)