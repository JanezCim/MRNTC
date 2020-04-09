# Absolute trajectory error calculator (also includes CPU calculator)

Takes in two tranformations and calculates euclidian distance between them. It writes the result with the ROS timestamp into a file. The timestamp is pulled from the first transform message. 

Package also includes cpu calculator that gets CPU usage and writes it to a file with a current ROS timestamp

# Developer notes

## Process of filtering out only the necesarry frames from the bag file

### 1. Using the rosbag_editor (this was actually used)

Instalation guide: https://github.com/facontidavide/rosbag_editor

### 2. Using rosbag_filter (everything did not quite work)

Deletes the frame base_footprint from the tf topic

    rosbag filter hokuyo_realsense_cartographer.bag tmp.bag "topic != '/tf' or (len(m.transforms)>0 and m.transforms[0].child_frame_id!='base_footprint')"

Deletes every other frame, but map->odom->base_footprint2

    rosbag filter input.bag output.bag "topic != '/tf' or (len(m.transforms)>0 and m.transforms[0].header.frame_id=='map')"

Deletes every other tf frame but map->odom->base_footprint and odom->base_footprint2

    rosbag filter input.bag output.bag "topic != '/tf' or (len(m.transforms)>0 and m.transforms[0].header.frame_id=='map' or m.transforms[0].header.frame_id=='odom')"

Split bag file (greater-than-or-equal-to (>=) and less-than-or-qual-to (<=) operators to keep all data before or after your cutoff point)

    rosbag filter input.bag output.bag "t.secs <= 1284703931.86"

Renaming the tf topic inside bag

    rosrun rosbag topic_renamer.py <in topic> <in bag> <out topic> <out bag>


    rosbag filter input.bag output.bag "topic != '/tf' or ((len(m.transforms)>0 and m.transforms[0].header.frame_id=='odom') and (len(m.transforms) <= 1 or (len(m.transforms) > 1 and m.transforms[1].header.frame_id = 'odom')))"

    rosbag filter original.bag filtered.bag "topic != '/tf' or ((len(m.transforms) > 0 and m.transforms[0].header.frame_id != 'odom') and (len(m.transforms) <= 1 or (len(m.transforms) > 1 and m.transforms[1].header.frame_id != 'odom')) and (len(m.transforms) <= 2 or (len(m.transforms) > 2 and m.transforms[2].header.frame_id != 'odom')) and (len(m.transforms) <= 3 or (len(m.transforms) > 3 and m.transforms[3].header.frame_id != 'odom')) and (len(m.transforms) <= 4 or (len(m.transforms) > 4 and m.transforms[4].header.frame_id != 'odom')))"