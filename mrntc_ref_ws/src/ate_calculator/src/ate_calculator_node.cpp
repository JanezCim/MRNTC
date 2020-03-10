// ATE = absolute trajectory error

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <fstream>


using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "ate_calculator");

  ros::NodeHandle node;


  //#####################################PARAMS################################################################### 
  string reference_target_frame;
  string test_target_frame;
  string reference_source_frame;
  string test_source_frame;
  node.param<std::string>("/ate_calculator/reference_target_frame", reference_target_frame, "ref_t_frame");
  node.param<std::string>("/ate_calculator/test_souce_frame", test_source_frame, "test_s_frame");
  node.param<std::string>("/ate_calculator/reference_source_frame", reference_source_frame, "ref_s_frame");
  node.param<std::string>("/ate_calculator/test_target_frame", test_target_frame, "test_t_frame");
  
  double timestamp_time_difference_thresh; // seconds
  node.param("/ate_calculator/timestamp_time_difference_thresh", timestamp_time_difference_thresh, 0.01);

  string out_file;
  node.param<std::string>("/ate_calculator/out_file_path", out_file, "output.txt");
 
  //#############################################################################################################


  ROS_INFO("Comparing transformation %s -> %s \n with transformation %s -> %s", reference_source_frame.c_str(),
                                                                                reference_target_frame.c_str(),
                                                                                test_source_frame.c_str(),
                                                                                test_target_frame.c_str());

  ROS_INFO("Timestamp time difference thresh is set to %f", timestamp_time_difference_thresh);                                                                                
  ROS_INFO("Writing results to file: %s", out_file.c_str());                                                                                
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // set the log file
  ofstream myfile(out_file.c_str());

  // innit the vars used in the program
  string output_text;
  double dx, dy, dist_diff;
  bool print_logging_msg = true;

  // main loop
  ros::Rate rate(10.0);
  while (node.ok()){
    // Innit the transforms
    geometry_msgs::TransformStamped refTransform;
    geometry_msgs::TransformStamped testTransform;
    
    // get both transformations, if you dont get them, warn and skip loop
    try{
      refTransform = tfBuffer.lookupTransform(reference_target_frame, reference_source_frame, ros::Time(0)); //target_frame = child_frame, source_frame = parent_frame
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      print_logging_msg = true;
      continue;
    }
    try{
      testTransform = tfBuffer.lookupTransform(test_target_frame, test_source_frame, ros::Time(0)); //target_frame = child_frame, source_frame = parent_frame
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      print_logging_msg = true;
      continue;
    }

    if(print_logging_msg){
      ROS_INFO("Got all transformations and logging now into %s ...", out_file.c_str());
      print_logging_msg = false;
    }

    // check the timestamp difference between the two messages
    ros::Duration diff=testTransform.header.stamp-refTransform.header.stamp;
    if(fabs(diff.toSec())>timestamp_time_difference_thresh){
      ROS_WARN("There are timestamp differences between tranformations larger than %f. Not counting towards ATE anymore", timestamp_time_difference_thresh);
    }
    else{
      dx = fabs(refTransform.transform.translation.x-testTransform.transform.translation.x);
      dy = fabs(refTransform.transform.translation.y-testTransform.transform.translation.y);
      dist_diff = sqrt(dx*dx+dy*dy);

      
      // logging difference and 
      myfile << dist_diff << "\t";
      myfile << testTransform.header.stamp;
      myfile << endl;
    }

    rate.sleep();
  }

  myfile.close();
  
  return 0;
};