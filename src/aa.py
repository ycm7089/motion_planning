#! /usr/bin/env python3

from cmath import sqrt
from math import pi
import rospy
import numpy as np
# from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import tf

def get_quaternion_from_euler(roll, pitch, yaw):

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class Motion :
    def __init__(self) :
        
        self.br = tf.TransformBroadcaster()

    def robot_pose
    
    def robot_pose_tf(self):
        # quaternion = get_quaternion_from_euler(0.0, 0.0, pi/3)

        self.br.sendTransform((0.0, 0.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, pi/3), rospy.Time.now(), "robot", "map")

        self.br.sendTransform((10.0, 5.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "goal", "map")
    def cal(self):
        to_goal_distance = sqrt(10*10 + 5* 5)

if __name__ == '__main__':
    rospy.init_node('Motion_Planning')
    motion_planning = Motion()

    while not rospy.is_shutdown():        
        motion_planning.robot_pose_tf()
       
    rospy.spin()
