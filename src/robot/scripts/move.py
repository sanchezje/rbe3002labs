#!/usr/bin/env python
from __future__ import print_function

import math
import copy
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
from tf.transformations import euler_from_quaternion



class Robot:

    def __init__(self):
        """"
        Set up the node here

        """
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.goal_sub = rospy.Subscriber('goal', PoseStamped, self.nav_to_pose)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.node = rospy.init_node('movememt', anonymous=True)

        self.px = 0
        self.py = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.speed = 0.1 # 10cm/s linear velocity
        self.omega = 1 # 1 rad/s angular velocity



    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a BLOCKING callback function. It should extract data from goal,
        drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
        # goal will be a PoseStamped message
        # we need to extract a position and a rotation from it
        goal_pose = goal.pose
        goal_posn = goal_pose.position #x y z

        quat = goal_pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        goal_rots = euler_from_quaternion(q) # 0=yaw 1=pitch 2=roll
        goal_rot = goal_rots[0]

        # determine angle to spin from start to line to goal
        # relative to the global coordinate system (so 0-2pi)
        path_angle = math.atan2(  (goal_posn.y - self.py) , (goal_posn.x - self.px))

        # rotate to face the goal vector

        self.rotate(path_angle - self.yaw)

        # determine euclidian distance from start to goal
        # using pythagorean theorem
        goal_dist = sqrt( (goal_posn.x - self.px)^2 + (goal_posn.y - self.py)^2  )

        # drive in a striaght line the appropriate distance
        self.drive_straight(speed, goal_dist)

        # rotate to satisfy the required final pose
        self.rotate(goal_rot - self.yaw) #note: yaw *should be* path_angle




    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive straight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
        """
        # looks like we're going to do an open loop drive
        start_time = ros.Time.now()
        leg_time = distance/speed

        while ros.Time.now() - start_time < leg_time:
            msg = Twist()
            msg.linear.x = speed
            self.cmd_pub.publish(msg)




    def rotate(self, angle):
        """
        Rotate in place
        to the specified angle relative to the global coordinate system
        :param angle: angle to rotate
        :return:
        """
        # check starting Odometry
        # figure out which way to turn

        delta_angle = angle - self.yaw

        if delta_angle > 0:
            # we want positive rotation
            self.omega = abs(self.omega)

        if delta_angle < 0:
            # we want negative rotation
            self.omega = - abs(self.omega)

        while abs(angle - self.yaw) > 0.05: #about a 3deg threshold
            msg = Twist()
            # @TODO going to guess angular.x corresponds to yaw, need to verify
            msg.angular.x = self.omega
            self.cmd_pub.publish(msg)



    def odom_callback(self, msg):

        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        # consume odometry message and extract useful info
        # extract the x and y position we're at
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        # extract the quaternion and make it usable
        quat = msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(q)



if __name__ == '__main__':
    r = Robot()
    try:
        rospy.spin() #exist always, only do things if
        #recieve messages to do so via callbacks
        # x = r.yaw
        # print(x)

    except:
        pass
