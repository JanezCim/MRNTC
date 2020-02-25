# Mobile Robot Navigation Techniques Comparison 

ROS: Kinetic

Linux: 16.04


# To run cartographer mapping

On test PC (every new terminal must be sourced in workspace and have ROS master ip setup):

    roslaunch mrntc_test_launch hokuyo_slam.launch
    roslaunch mrntc_test_launch rs_t265.launch

On ref PC (every new terminal must be sourced in workspace and have ROS master ip setup):

    roslaunch mrntc_ref_launch cartographer.launchroslaunch mrntc_ref_launch cartographer.launch