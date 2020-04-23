// Logs the 2D coordinates: x, y, yaw of a transform "parent_frame"->"child_frame" into a file "out_file_path" with rate "frequency"

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "log_coordinates");

  ros::NodeHandle node("~");

  //#####################################ROS PARAMS################################################################ 
  string parent_frame;
  string child_frame;
  node.param<std::string>("parent_frame", parent_frame, "undefined_parent_frame");
  node.param<std::string>("child_frame", child_frame, "undefined_child_frame");
  
  double frequency; //hz
  node.param("frequency", frequency, 5.0);

  string out_file;
  node.param<std::string>("out_file_path", out_file, "coordinates_output.txt"); 
  //#############################################################################################################

  ROS_INFO("Logging coordinates with frames %s -> %s with frequency %lf",parent_frame.c_str(),
                                                                        child_frame.c_str(),
                                                                        frequency);


  ROS_INFO("Otput to be written to: %s", out_file.c_str());                                                                                
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // set the log file
  ofstream myfile(out_file.c_str());

  // innit the vars used in the program
  // X AND Y PRESENT THE COORDINATES AND Z PRESENTS THE ORIENTATION AROUND Z AXIS
  string output_text;
  double old_x = 0;
  double old_y = 0;
  double old_z = 0;

  double x,y,z;

  // main loop
  ros::Rate rate(frequency);
  while (node.ok()){
    geometry_msgs::TransformStamped trans;
    
    try{
      trans = tfBuffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
      x = trans.transform.translation.x;
      y = trans.transform.translation.y;
      z = trans.transform.translation.z;
      
      ROS_INFO("Logging coordinate %lf, %lf, %lf", x,y,z);

      // logging coordinates 
      myfile << x << "\t";
      myfile << y << "\t";
      myfile << z << "\t";
      myfile << ros::Time::now() << "\t";
      myfile << endl;

      old_x = x;
      old_y = y;
      old_z = z;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }

  myfile.close();
  
  return 0;
};