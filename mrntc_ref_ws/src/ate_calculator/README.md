# Absolute trajectory error calculator

Takes in the current position of the robot adn 

# Developer notes

## Process of filtering out only the necesarry frames from the bag file

The result of these two commands is that only map->odom->base_footprint2 transforms exsist

Deletes the frame base_footprint from the tf topic

    rosbag filter hokuyo_realsense_cartographer.bag tmp.bag "topic != '/tf' or (len(m.transforms)>0 and m.transforms[0].child_frame_id!='base_footprint')"

Deletes every other frame, but map->odom->base_footprint2

rosbag filter tmp.bag hokuyo_realsense_cartographer_filtered_frames.bag "topic != '/tf' or (len(m.transforms)>0 and m.transforms[0].header.frame_id=='map')"