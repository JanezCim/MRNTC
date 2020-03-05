# REFERENCE PC WORKSPACE

This workspace is ment to be uploaded and compiled on the reference PC

# Reference PC setup log

CARTOGRAPHER

Followed the official tutorial of instalation: https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html
[When compiling the protobuf, the git repo was not accesable from robot (probably because of bad internet connection - it had to be cloned to a PC and then transfered to the robot via sftp)]
All the files neceserry to run cartographer are contained within this workspace, so the only thin needed is to compile it.

Rtabmap

    sudo apt install ros-kinetic-rtabmap