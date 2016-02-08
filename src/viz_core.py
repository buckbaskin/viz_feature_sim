#!/usr/bin/env python

'''
Implement a ROS node that simulates using a 360 degree camera to identify unique
features and patterns in the surrounding environment and track their bearing
from the camera. The simulator works at a level below the camera, and publishes
the bearings and colors.

It is currently just publishing the exact measurement and color.
'''

import rospy
import math

from nav_msgs.msg import Odometry
from viz_feature_sim.msg import Observation

from collections import namedtuple
from random import random as random_double
from tf import transformations as tft

Feature = namedtuple('Feature', 'x y size red green blue frame')

def quaternion_to_heading(quat):
    '''
    Convert a quaternion to a yaw-heading
    '''
    try:
        quat = [quat.x, quat.y, quat.z, quat.w]
    except AttributeError:
        quat = quat
    yaw = tft.euler_from_quaternion(quat)[2]
    return yaw

def easy_feature(x, y, size=.2, color=None):
    '''
    helper method to create features to use as navigation references
    '''
    # red, green, blue = color
    if color is None:
        color = (random_double(), random_double(), random_double())
    red, green, blue = color
    return Feature(x, y, size, red, green, blue, frame='/map')

def easy_observation(odom, feature):
    '''
    helper method to create Observations to use in navigation calculations
    '''
    observ = Observation()
    observ.header.stamp = odom.header
    heading = math.atan2(odom.pose.pose.position.y-feature.y,
        odom.pose.pose.position.x-feature.x)
    observ.bearing = heading - quaternion_to_heading(odom.pose.pose.orientation)
    observ.color = feature.color
    return observ

class CamSim(object):
    '''
    CamSim is the python class that is created to simulate visual features in
     the simulator
    '''
    def __init__(self):
        rospy.init_node('CamSim')
        self.feature_list = []
        self.feature_list.append(easy_feature(0, 0))
        self.feature_list.append(easy_feature(0, 10))
        self.feature_list.append(easy_feature(10, 0))
        self.feature_list.append(easy_feature(10, 10))
        self.true_pose = rospy.Subscriber('/base_pose_ground_truth', Odometry,
            self.process_position)
        self.feature_pub = rospy.Publisher('/camera/features', Observation,
            queue_size=10)
        rospy.loginfo('CamSim aka viz_core aka viz_feature_sim ready')
        rospy.spin()

    def process_position(self, odom):
        '''
        ros callback for the position that publishes new sensor data for that
         position
        '''
        # odom.header.frame_id = '/map'
        for feature in self.feature_list:
            self.feature_pub.publish(easy_observation(odom, feature))


if __name__ == '__main__':
    CamSim()
