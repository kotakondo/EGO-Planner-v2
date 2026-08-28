#!/usr/bin/env python3
"""Broadcast TF from odometry so RViz camera can follow the drone."""
import rospy
import tf
from nav_msgs.msg import Odometry

def cb(msg):
    br.sendTransform(
        (msg.pose.pose.position.x,
         msg.pose.pose.position.y,
         msg.pose.pose.position.z),
        (msg.pose.pose.orientation.x,
         msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w),
        msg.header.stamp,
        "drone_0",
        "world")

rospy.init_node("odom_to_tf", anonymous=True)
br = tf.TransformBroadcaster()
rospy.Subscriber("/drone_0_visual_slam/odom", Odometry, cb)
rospy.spin()
