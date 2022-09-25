#! /usr/bin/env python3

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
        
        self.arrow_pub = rospy.Publisher("/robot_position", Marker, queue_size=10)

        self.robot_position = Marker()
    
    def robot_pose_tf(self):
        quaternion = get_quaternion_from_euler(0.0, 0.0, pi/3)

        self.robot_position.header.frame_id = "map"
        self.robot_position.header.stamp = rospy.Time.now()
        self.robot_position.ns = "arrow"
        
        self.robot_position.type = Marker.ARROW
        self.robot_position.action = Marker.ADD

        self.robot_position.pose.position.x = 0.0
        self.robot_position.pose.position.y = 0.0
        self.robot_position.pose.position.z = 0.0
        
        self.robot_position.pose.orientation.x = quaternion[0]
        self.robot_position.pose.orientation.y = quaternion[1]
        self.robot_position.pose.orientation.z = quaternion[2]
        self.robot_position.pose.orientation.w = quaternion[3]

        self.robot_position.scale.x = 1
        self.robot_position.scale.y = 1
        self.robot_position.scale.z = 1

        self.robot_position.color.r = 1.0
        self.robot_position.color.g = 0.0
        self.robot_position.color.b = 0.0
        self.robot_position.color.a = 1.0       

        # self.robot_position.points.append(Point(0.0, 0.0, 0.0))
        # self.robot_position.points.append(Point(3.0, 2.0, 1.0))
        # print(self.robot_position)
        self.arrow_pub.publish(self.robot_position)

        print(self.arrow_pub)

if __name__ == '__main__':
    rospy.init_node('Motion_Planning')

    motion_planning = Motion()
    motion_planning.robot_pose_tf()
       
    rospy.spin()
