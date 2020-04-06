 
# How to setup the Youbot motor driver

ROS: kinetic
Linux: 16.04

    sudo sudo apt install ros-kinetic-youbot-driver

Edit the youbot ethercat config folder

    nano /opt/ros/kinetic/share/youbot_driver/config/youbot-ethercat.cfg

And inside it, change the parameter *EthernetDevice* to the correct interace - that can be found using comand *ifconfig*

Create a separate workspace and clone the youbot ros interface, youbot description, checkout to devel branch and compile with catkin_make

    #inside dedicated workspace/src

    cd <workspace_dir>/src
    git clone https://github.com/youbot/youbot_driver_ros_interface.git
    git clone https://github.com/youbot/youbot_description.git
    cd youbot_description/
    git checkout kinetic-devel
    cd ..
    cd ..
    catkin_make
    source devel/setup.bash

The permissions for ethercat also have to be made

    sudo setcap cap_net_raw+ep <workspace_dir>/devel/lib/youbot_driver_ros_interface/youbot_driver_ros_interface
    sudo ldconfig <workspace_dir>/devel/lib/youbot_driver_ros_interface

from the same directory, youbot driver can be launched

    roslaunch youbot_driver_ros_interface youbot_driver.launch


Usefull links:

Permission problems with the ethercat acces, follow the directions here: http://www.youbot-store.com/wiki/index.php/Execute_as_a_root

