#!/bin/bash

#sources ROS. Add appropriate distro: source "/opt/ros/<DISTRO/setup.bash"
#source "/opt/ros/kinetic/setup.bash"

#############################FINDING THE LOCAL IP ADRESS (only one must be uncommented)##########################

# Detects the IP of local machine (localhost does not always work)

# This takes the last ip adress which could cause problems if local machine has multiple network config setup
# ipv4=$(ip addr show |grep -w inet |tail -1 |awk '{ print $2}' | cut -d '/' -f 1)

#This takes the IP adress that starts with 10
ipv4=$(ip addr show |grep -w inet | grep -w 10 |awk '{ print $2}' | cut -d '/' -f 1)

#This takes the IP adress of a certain network interface
# ipv4=$(/sbin/ifconfig enx9cebe8162826 | grep 'inet addr:' | cut -d: -f2| cut -d' ' -f1)


#############################master IP setup (only one of the following must be uncommented)####################################
#exports master URI -> on local machine
#export ROS_MASTER_URI=http://$ipv4:11311

#exports master URI on other machine, wher you specify IP of machine master is on
export ROS_MASTER_URI=http://10.42.0.1:11311
################################################################################################################################

#sets your ROS_IP
export ROS_IP=$ipv4

#sets your hostname
export ROS_HOSTNAME=$ipv4
