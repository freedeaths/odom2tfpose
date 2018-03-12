#! /usr/bin/env python2.7

import rospy

import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg


class OdometryTransformer(object):
    def __init__(self):
        rospy.init_node('odometry2tfpose')
        self.sub = rospy.Subscriber('/gps/rtkfix', nav_msgs.msg.Odometry, callback=self.odometry_cb)

        # Publishes vehicle position in the map frame
        # QUESTION: Does this have to be in the map frame, or can we give it in GPS frame
        # and assume that the subsriber properly transforms it?
        self.pose_pub = rospy.Publisher('/gnss_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
        # Publishes whether fix is valid
        # QUESTION: Is this required? Or do they only check for False?
        # QUESTION: Does the autoware 
        self.stat_pub = rospy.Publisher('/gnss_stat', std_msgs.msg.Bool)

    def odometry_cb(self, msg):

        pose_msg = geometry_msgs.msg.PoseStamped()

        pose_msg.header = msg.header
        pose_msg.pose.position = msg.pose.pose.position
        
        self.pose_pub.publish(pose_msg)
        self.stat_pub.publish(std_msgs.msg.Bool(True))

if __name__=="__main__":
    transformer = OdometryTransformer()
    rospy.spin()
