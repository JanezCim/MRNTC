#!/bin/bash

#sources ROS. Add appropriate distro: source "/opt/ros/<DISTRO/setup.bash"
source "/opt/ros/kinetic/setup.bash"

#detects the IP of local machine (localhost does not always work)
#This could cause problems if local machine has multiple network config setup
ipv4=$(ip addr show |grep -w inet |tail -1 |awk '{ print $2}' | cut -d '/' -f 1)


#############################master IP setup (only one of the following must be uncommented)####################################
#exports master URI -> on local machine
#export ROS_MASTER_URI=http://$ipv4:11311

#exports master URI on other machine, wher you specify IP of machine master is on
export ROS_MASTER_URI=http://192.168.0.102:11311
################################################################################################################################

#sets your ROS_IP
export ROS_IP=$ipv4

#sets your hostname
export ROS_HOSTNAME=$ipv4