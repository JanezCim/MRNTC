#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "ate_calculator");

  ros::NodeHandle node;

  string reference_target_frame;
  string test_target_frame;
  string reference_source_frame;
  string test_source_frame;
  node.param<std::string>("/ate_calculator/reference_target_frame", reference_target_frame, "ref_t_frame");
  node.param<std::string>("/ate_calculator/test_souce_frame", test_source_frame, "test_s_frame");
  node.param<std::string>("/ate_calculator/reference_source_frame", reference_source_frame, "ref_s_frame");
  node.param<std::string>("/ate_calculator/test_target_frame", test_target_frame, "test_t_frame");

  ROS_INFO("Comparing transformation %s -> %s \n with transformation %s -> %s", reference_source_frame.c_str(),
                                                                                reference_target_frame.c_str(),
                                                                                test_source_frame.c_str(),
                                                                                test_target_frame.c_str());

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped refTransform;
    geometry_msgs::TransformStamped testTransform;
    try{
      refTransform = tfBuffer.lookupTransform(reference_target_frame, reference_source_frame, ros::Time(0)); //target_frame = child_frame, source_frame = parent_frame
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // ROS_INFO("REF: x: %lf, y %lf", refTransform.transform.translation.x, refTransform.transform.translation.y);

    try{
      testTransform = tfBuffer.lookupTransform(test_target_frame, test_source_frame, ros::Time(0)); //target_frame = child_frame, source_frame = parent_frame
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // ROS_INFO("TEST: x: %lf, y %lf", testTransform.transform.translation.x, testTransform.transform.translation.y);

    double dx = fabs(refTransform.transform.translation.x-testTransform.transform.translation.x);
    double dy = fabs(refTransform.transform.translation.y-testTransform.transform.translation.y);

    double ate = sqrt(dx*dx+dy*dy);

    ROS_INFO("ATE: %lf", ate);


    rate.sleep();
  }
  return 0;
};