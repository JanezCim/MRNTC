#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np

pub = rospy.Publisher('/odom_republished', Odometry, queue_size=10) 

def callback(data):
  odom_cov = data
  pose_cov = (1e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 
              0.0, 1e-3, 0.0, 0.0, 0.0, 0.0, 
              0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 
              0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 1e-3)

  twist_cov =(0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 
              0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 
              0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 
              0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 
              0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 
              0.0, 0.0, 0.0, 0.0, 0.0, 0.01)

  odom_cov.pose.covariance = tuple(pose_cov)
  odom_cov.twist.covariance = tuple(twist_cov)
  pub.publish(odom_cov)

if __name__ == "__main__":
    rospy.init_node('odom_republisher')
    rospy.Subscriber('/odom', Odometry, callback)
    rospy.spin()    