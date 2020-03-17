# Aruco markers folder

Contains Files needed to simulate the markers in gazebo (needed if the simulation of the aruco_slam) - size of the markers in gazebo is 14cm

If PDF for printable markers is needed, it can be obtained with this command (generates markers of size 14cm):

    rosrun aruco_detect create_markers.py 100 112 fiducials.pdf