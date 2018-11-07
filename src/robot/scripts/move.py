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
    # omega = None

    def __init__(self):
        """"
        Set up the node here

        """

        self.px = 0
        self.py = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.speed = 0.1 # 10cm/s linear velocity
        self.omega = 1 # 1 rad/s angular velocity

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.nav_to_pose)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.node = rospy.init_node('movement', anonymous=True)



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
        goal_rot = goal_rots[2] # this will be the final yaw

        # determine angle to spin from start to line to goal
        # relative to the global coordinate system (so -pi to pi because atan2)
        path_angle = math.atan2(  (goal_posn.y - self.py) , (goal_posn.x - self.px))

        # rotate to face the goal vector
        diff = path_angle - self.yaw
        self.rotate(path_angle) # rotate to the specified angle

        # determine euclidian distance from start to goal
        # using pythagorean theorem
        goal_dist = math.sqrt( pow((goal_posn.x - self.px),2) + pow((goal_posn.y - self.py), 2)  )

        # print("goal dist = ", goal_dist)

        # drive in a striaght line the appropriate distance
        self.drive_straight(self.speed, goal_dist)

        # # rotate to satisfy the required final pose
        self.rotate(goal_rot) #note: yaw *should be* path_angle
        msg = Twist()
        self.cmd_pub.publish(msg)





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
        start_time = rospy.get_time()
        leg_time = distance/speed

        while rospy.get_time() - start_time < leg_time:
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

        #diff is a quantity between -2pi and 2pi
        diff = angle - self.yaw

        if diff > math.pi:
            diff = diff - 2*math.pi
        elif diff < -1*math.pi:
            diff = diff + 2*math.pi
        else:
            pass
        # assign the proper direction to rotate
        self.omega = math.copysign(self.omega, diff)
        msg = Twist()
        # pack a twist message and send it
        # angular.z corresponds to yaw. I dun messed up
        msg.angular.z = self.omega
        self.cmd_pub.publish(msg)

        while abs(angle - self.yaw) > 0.05: #about a 3deg threshold


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
    except:
        pass
