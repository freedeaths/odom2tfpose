#! /usr/bin/env python2.7

import rospy
import tf

import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg


class OdometryTransformer(object):
    def __init__(self):
        rospy.init_node('odometry2tfpose')
        self.sub = rospy.Subscriber('/gps/rtkfix', nav_msgs.msg.Odometry, callback=self.odometry_cb)

        # Publishes vehicle position in the map frame
        # QUESTION: Does this have to be in the map frame, or can we give it in GPS frame?
        #           I looked at ndt_estimation, and didn't see it even checking the
        #           frame in the header...
        self.pose_pub = rospy.Publisher('/gnss_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
        # Publishes whether fix is valid
        # QUESTION: Is this required? The autoware config files seem to expect it,
        #           but I couldn't find it anywhere in the code.
        self.stat_pub = rospy.Publisher('/gnss_stat', std_msgs.msg.Bool, queue_size=1)

        self.tf = tf.Transformer(True, rospy.Duration(5.0))

    def odometry_cb(self, msg):

        pose_msg = geometry_msgs.msg.PoseStamped()

        pose_msg.header = msg.header
        pose_msg.pose.position = msg.pose.pose.position

        target_frame = "map"
        if self.tf.canTransform(target_frame, pose_msg.header.frame_id, pose_msg.header.stamp):
            transformed_msg = self.tf.transformPose(target_frame, pose_msg)
            self.pose_pub.publish(transformed_msg)
            self.stat_pub.publish(std_msgs.msg.Bool(True))

if __name__=="__main__":
    transformer = OdometryTransformer()
    rospy.spin()
