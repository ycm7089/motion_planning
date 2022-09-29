#!/usr/bin/env python3

import rospy
from math import *
import tf

def tf_pub():
    br = tf.TransformBroadcaster()

    br.sendTransform((0.0, 0.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, pi/3), rospy.Time.now(), "robot", "map")
    br.sendTransform((10.0, 5.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "goal", "map")

def main():
    rospy.init_node('tf_publisher')
    while not rospy.is_shutdown():
        tf_pub()
    


if __name__ == '__main__':
    main()