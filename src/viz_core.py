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
import random
random.seed('This is a long string, but it is hashable')

from nav_msgs.msg import Odometry
from viz_feature_sim.msg import Blob, VizScan

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
        color = (random_double()*255, random_double()*255, random_double()*255)
    red, green, blue = color
    return Feature(x, y, size, red, green, blue, frame='/map')

def easy_observation(odom, feature):
    '''
    helper method to create Blobs to use in navigation calculations
    '''
    blob = Blob()
    heading = math.atan2(feature.y-odom.pose.pose.position.y,
        feature.x-odom.pose.pose.position.x)
    blob.bearing = heading - quaternion_to_heading(odom.pose.pose.orientation)
    rospy.loginfo('h | b || %f | %f' % (heading, quaternion_to_heading(odom.pose.pose.orientation),))
    blob.size = feature.red
    blob.color.r = feature.red
    blob.color.g = feature.green
    blob.color.b = feature.blue
    return blob

class CamSim(object):
    '''
    CamSim is the python class that is created to simulate visual features in
     the simulator
    '''
    def __init__(self):
        rospy.init_node('CamSim')
        self.num_features = 5
        self.feature_list = []
        self.feature_list.append(easy_feature(0, 0, color=(161, 77, 137,)))
        self.feature_list.append(easy_feature(10, 0, color=(75, 55, 230,)))
        self.feature_list.append(easy_feature(0, 15, color=(82, 120, 68,)))
        self.feature_list.append(easy_feature(10, 15, color=(224, 37, 192,)))
        while(len(self.feature_list)) < self.num_features:
            x = random.randint(-20, 35)
            y = random.randint(-20, 35)
            self.feature_list.append(easy_feature(x, y))

        self.true_pose = rospy.Subscriber('/base_pose_ground_truth', Odometry,
            self.process_position)
        self.feature_pub = rospy.Publisher('/camera/features', VizScan,
            queue_size=10)
        rospy.loginfo('CamSim aka viz_core aka viz_feature_sim ready')
        self.rateLimit = rospy.Rate(10)

    def run(self):
        rospy.spin()

    def process_position(self, odom):
        '''
        ros callback for the position that publishes new sensor data for that
         position
        '''

        vs = VizScan()
        vs.header = odom.header
        vs.observes = []

        o_x = odom.pose.pose.position.x
        o_y = odom.pose.pose.position.y
        o_h = quaternion_to_heading(odom.pose.pose.orientation)

        # rospy.loginfo('odom: %f %f %f' % (o_x, o_y, o_h,))

        # odom.header.frame_id = '/map'
        for feature in self.feature_list:
            # rospy.loginfo('feat: %f %f' % (feature.x, feature.y,))
            compass = math.atan2(feature.y-o_y, feature.x-o_x)
            bearing = compass - o_h
            # rospy.loginfo('compass: %f heading: %f bearing: %f' % (compass, o_h, bearing))
            bob = Blob()
            bob.bearing = bearing
            bob.color.r = feature.red
            bob.color.g = feature.green
            bob.color.b = feature.blue
            vs.observes.append(bob)

        rospy.loginfo('viz_core publish sensor data')
        self.feature_pub.publish(vs)
        self.rateLimit.sleep()




if __name__ == '__main__':
    cs = CamSim()
    cs.run()
