#! /usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def posecb(msg):
    print(msg)

    
    local_path.header.stamp = rospy.Time.now()
    local_path.header.frame_id = "map"
    local_path.poses.append(msg)

    path_pub.publish(local_path)
 
if __name__ == '__main__':

    rospy.init_node('Pub_rviz')
    local_path = Path()
    cur_robot_pose_sub = rospy.Subscriber("/current_pose",PoseStamped, posecb)
    path_pub = rospy.Publisher("/robot_path", Path, queue_size= 1)

    rospy.spin()


