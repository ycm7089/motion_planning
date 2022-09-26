#! /usr/bin/env python3

#pythonrobotics -> control 아마 내가 TF 만들고 움직이는거 보여주면 될듯

from ctypes.wintypes import POINT
from math import *
import rospy
import numpy as np
# from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point
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
        self.points_pub = rospy.Publisher("/goal_position", Marker, queue_size=10)

        self.robot_position = Marker()
        self.goal_position = Marker()
        self.cal_to_goal = PoseStamped()
    
    def robot_pose(self):
        quaternion = get_quaternion_from_euler(0.0, 0.0, pi/3)

        self.robot_position.header.frame_id = "map"
        self.robot_position.header.stamp = rospy.Time.now()

        self.robot_position.type = Marker.ARROW
        self.robot_position.action = Marker.ADD

        self.robot_position.pose.position.x = 0.0
        self.robot_position.pose.position.y = 0.0
        self.robot_position.pose.position.z = 0.0
        
        self.robot_position.pose.orientation.x = quaternion[0]
        self.robot_position.pose.orientation.y = quaternion[1]
        self.robot_position.pose.orientation.z = quaternion[2]
        self.robot_position.pose.orientation.w = quaternion[3]

        self.robot_position.scale.x = 1.0
        self.robot_position.scale.y = 0.2
        self.robot_position.scale.z = 0.1

        self.robot_position.color.r = 1.0
        self.robot_position.color.g = 0.0
        self.robot_position.color.b = 0.0
        self.robot_position.color.a = 1.0      

        self.arrow_pub.publish(self.robot_position)

    def goal_pose(self):
        self.goal_position.header.frame_id = "map"
        self.goal_position.header.stamp = rospy.Time.now()

        self.goal_position.type = Marker.SPHERE
        self.goal_position.action = Marker.ADD

        self.goal_position.pose.position.x = 10.0
        self.goal_position.pose.position.y = 5.0
        self.goal_position.pose.position.z = 0.0

        self.goal_position.scale.x = 0.3
        self.goal_position.scale.y = 0.3
        self.goal_position.scale.z = 0.3

        self.goal_position.color.r = 1.0
        self.goal_position.color.g = 0.0
        self.goal_position.color.b = 0.0
        self.goal_position.color.a = 1.0    

        # self.goal_position.points.append(Point(10.0, 5.0, 0.0))  
        self.points_pub.publish(self.goal_position)

    def cal_robot(self):
        to_goal_distance = sqrt((self.robot_position.pose.position.x - self.goal_position.pose.position.x)**2+ (self.robot_position.pose.position.y - self.goal_position.pose.position.y)**2)
        print(to_goal_distance)
        # self.cal_to_goal.po
if __name__ == '__main__':
    rospy.init_node('Motion_Planning')

    motion_planning = Motion()
    while not rospy.is_shutdown():
        motion_planning.robot_pose()
        motion_planning.goal_pose()
        motion_planning.cal_robot()
       
    rospy.spin()
