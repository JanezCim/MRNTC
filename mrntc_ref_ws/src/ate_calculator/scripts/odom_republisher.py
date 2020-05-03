#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np

pub = rospy.Publisher('/odom_republished', Odometry, queue_size=10) 
to_override_pose_covariance = False
to_override_twist_covariance = False
t_cov = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #twist covariance diagonal override
p_cov = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #pose covariance diagonal override

def callback(data):
  odom_cov = data

  if(to_override_pose_covariance):
    pose_cov = (p_cov[0], 0.0, 0.0, 0.0, 0.0, 0.0, 
                0.0, p_cov[1], 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.0, p_cov[2], 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, p_cov[3], 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, p_cov[4], 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, p_cov[5])
    odom_cov.pose.covariance = tuple(pose_cov)

  if(to_override_twist_covariance):
    twist_cov =(t_cov[0], 0.0, 0.0, 0.0, 0.0, 0.0, 
                0.0, t_cov[1], 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.0, t_cov[2], 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, t_cov[3], 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, t_cov[4], 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0, t_cov[5])
    odom_cov.twist.covariance = tuple(twist_cov)
  
  
  pub.publish(odom_cov)

if __name__ == "__main__":
    rospy.init_node('odom_republisher')
    rospy.Subscriber('/odom', Odometry, callback)
    
    if rospy.has_param("~pose_covariance_diagonal_override"):
      tmp = rospy.get_param("~pose_covariance_diagonal_override")
      rospy.loginfo("Overriding odometry pose covarance with "+tmp)
      tmp = [[float(x.strip(' ')) for x in ss.lstrip(' [,').split(', ')] for ss in tmp.rstrip(']').split(']')]
      p_cov = tmp[0]
      to_override_pose_covariance = True
    
    
    if rospy.has_param("~twist_covariance_diagonal_override"):
      tmp = rospy.get_param("~twist_covariance_diagonal_override")
      rospy.loginfo("Overriding odometry twist covarance with "+tmp)
      tmp = [[float(x.strip(' ')) for x in ss.lstrip(' [,').split(', ')] for ss in tmp.rstrip(']').split(']')]
      t_cov = tmp[0]
      to_override_twist_covariance = True

    rospy.spin()    