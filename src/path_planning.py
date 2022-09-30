#! /usr/bin/env python3

from concurrent.futures import thread
from math import *
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Twist, PoseStamped
import tf

class Motion :
    def __init__(self) :
        
        self.robot_pose = PoseStamped()
        self.goal_pose = Pose()

        self.theta_start = pi / 2

        self.k_rho = 3.0
        self.k_alpha = 5.0
        self.k_beta = -2.0

        self.max_linear_speed = 0.01
        self.max_angular_speed = 0.03

        self._is_reach_goal = False
        self.br = tf.TransformBroadcaster()
        # 로봇 포즈 pub
        # 골 지점 pub
        # 속도 pub
        self.robot_pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
        self.robot_cmd_vel = rospy.Publisher('robot_vel', Twist, queue_size= 1)

    def get_quaternion_from_euler(self, roll, pitch, yaw):

        self.qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        self.qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        self.qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        self.qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [self.qx, self.qy, self.qz, self.qw]

    def robot_loc(self) :
        robot_quaternion = self.get_quaternion_from_euler(0.0, 0.0, self.theta_start)

        self.robot_pose.header.frame_id = "map"
        self.robot_pose.header.stamp = rospy.Time.now()

        self.robot_pose.pose.position.x = -5.0
        self.robot_pose.pose.position.y = -3.0
        self.robot_pose.pose.position.z = 0.0

        self.robot_pose.pose.orientation.x = robot_quaternion[0]
        self.robot_pose.pose.orientation.y = robot_quaternion[1]
        self.robot_pose.pose.orientation.z = robot_quaternion[2]
        self.robot_pose.pose.orientation.w = robot_quaternion[3]

        goal_quaternion = self.get_quaternion_from_euler(0.0, 0.0, 0.0)

        self.goal_pose.position.x = 10.0
        self.goal_pose.position.y = 5.0
        self.goal_pose.position.z = 0.0

        self.goal_pose.orientation.x = goal_quaternion[0]
        self.goal_pose.orientation.y = goal_quaternion[1]
        self.goal_pose.orientation.z = goal_quaternion[2]
        self.goal_pose.orientation.w = goal_quaternion[3]

        self.move_to_goal(self.robot_pose, self.goal_pose)

    def move_to_goal (self, robot, goal):

        x_start = robot.pose.position.x
        y_start = robot.pose.position.y
        theta = self.theta_start

        x_goal = goal.position.x
        y_goal = goal.position.y
        theta_goal = 0.0
        
        self.delta_x = x_goal - x_start
        self.delta_y = y_goal - y_start
        dt = 0.01
        
        rho = sqrt( (self.delta_x)**2 + (self.delta_y)**2 )

        self.tf_pub(x_start, y_start, theta)

        while not self._is_reach_goal :
            if rho > 0.001 :

                self.delta_x = x_goal - x_start
                self.delta_y = y_goal - y_start
                
                rho = sqrt( (self.delta_x)**2 + (self.delta_y)**2 )
                alpha = -theta + np.arctan2(self.delta_y, self.delta_x)
                beta = -theta - alpha + theta_goal

                v,w = self.cal_vel(rho, alpha, beta)

                if abs(v) > self.max_linear_speed:
                    v = np.sign(v) * self.max_linear_speed

                if abs(w) > self.max_angular_speed:
                    w = np.sign(w) * self.max_angular_speed

                theta = theta + w * dt
                x_start = x_start + v * np.cos(theta) * dt
                y_start = y_start + v * np.sin(theta) * dt
                
                self.robot_pose.pose.position.x = x_start
                self.robot_pose.pose.position.y = y_start
                self.robot_pose.pose.orientation.w = theta

                self.robot_pose_pub.publish(self.robot_pose)
                # if self.robot_pose.pose.position.x < 0:
                print(self.robot_pose.pose.position.x)
                self.tf_pub(x_start, y_start, theta)

            else :
                v= 0.0
                w=0.0

                self._is_reach_goal = True
        
        if self._is_reach_goal :
            print("Goal Reached")
            self._is_reach_goal = False

    def cal_vel(self, rho, alpha, beta):
        rho = sqrt( (self.delta_x)**2 + (self.delta_y)**2 )

        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta
        # print(v,w)
        if alpha > pi /2 or alpha <= - pi /2 :
            v = - v
            w = - w

        return v,w

    def tf_pub (self, x, y, theta):
        
        self.br.sendTransform((x, y, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, theta), rospy.Time.now(), "robot", "map")
        self.br.sendTransform((10.0, 5.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "goal", "map")

if __name__ == '__main__':
    rospy.init_node('Motion_Planning')
    motion_planning = Motion()

    while not rospy.is_shutdown():        
        motion_planning.robot_loc()
       
    rospy.spin()
