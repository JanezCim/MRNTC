#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

pub = rospy.Publisher('/out_pose', PoseWithCovarianceStamped, queue_size=10) 

to_override_covariance = False
cov = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #override covariance diagonal

def callback(data):
  npose = data
  if(to_override_covariance):
    pose_cov = (cov[0], 0.0, 0.0, 0.0, 0.0, 0.0, 
                0.0, cov[1], 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.0, cov[2], 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, cov[3], 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, cov[4], 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, cov[5])

    npose.pose.covariance = tuple(pose_cov)
  pub.publish(npose)

if __name__ == "__main__":
    rospy.init_node('pose_republisher')
    rospy.Subscriber('/in_pose', PoseWithCovarianceStamped, callback)

    if rospy.has_param("~covariance_diagonal_override"):
      tmp = rospy.get_param("~covariance_diagonal_override")
      rospy.loginfo("Overriding pose covarance with "+tmp)
      tmp = [[float(x.strip(' ')) for x in ss.lstrip(' [,').split(', ')] for ss in tmp.rstrip(']').split(']')]
      cov = tmp[0]
      to_override_covariance = True

    rospy.spin()    