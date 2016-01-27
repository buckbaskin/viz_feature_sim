#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from viz_feature_sim.msg import Observation

from collections import namedtuple
from random import random as random_double
from tf import transformations as tft

Feature = namedtuple('Feature', 'x y size red green blue frame')

def quaternion_to_heading(quat):
    try:
        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    except AttriubteError:
        quat = quaternion
    yaw = tft.euler_from_quaternion(quat)[2]
    return yaw

def easy_feature(x, y, size=.2, color=None):
    # red, green, blue = color
    if color is None:
        color = (random_double(), random_double(), random_double())
    return Feature(x, y, size, *color, '/map')

def easy_observation(odom, feature):
    o = Observation()
    o.header.stamp = odom.header
    heading = math.atan2(odom.pose.pose.position.y-feature.y, odom.pose.pose.position.x-feature.x)
    o.bearing = heading - quaternion_to_heading(odom.pose.pose.orientation)
    o.color = feature.color
    return o

class CamSim(object):
    def __init__(self):
        rospy.init_node('CamSim')
        self.feature_list = []
        self.feature_list.append(0,0)
        self.feature_list.append(0,10)
        self.feature_list.append(10,0)
        self.feature_list.append(10,10)
        self.true_pose = rospy.Subscriber('/base_pose_ground_truth', Odometry, 
            self.process_position)
        self.feature_pub = rospy.Publisher('/camera/features', Observation)

    def process_position(self, odom):
        odom.header.frame_id = '/map'
        for feature in self.feature_list:
            self.feature_pub.publish(easy_observation(odom, feature))


if __name__ == '__main__':
    CamSim()
