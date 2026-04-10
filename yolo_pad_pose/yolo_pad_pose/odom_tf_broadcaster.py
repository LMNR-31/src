#!/usr/bin/env python3
"""Broadcast TF from a nav_msgs/Odometry topic.

MAVROS publishes nav_msgs/Odometry on /uav1/mavros/local_position/odom with
header.frame_id='uav1/map' and child_frame_id='uav1/base_link', but it does
NOT automatically write these frames to /tf.  This node fills that gap so the
TF tree is connected and tools like tf2_echo, RViz and tf2-based transforms
work correctly.

Usage
-----
ros2 run yolo_pad_pose odom_tf_broadcaster

Verify
------
ros2 run tf2_ros tf2_echo uav1/map uav1/base_link

Parameters
----------
odom_topic             : str  – Odometry topic to subscribe to.
                                Default: /uav1/mavros/local_position/odom
tf_parent_frame_override: str – Override parent frame id (default: use
                                msg.header.frame_id).
tf_child_frame_override : str – Override child frame id (default: use
                                msg.child_frame_id or 'base_link').
use_odom_header_stamp   : bool – When True (default) the published
                                TransformStamped carries the stamp from the
                                Odometry message; when False, rclpy.Time is
                                used (i.e. node's current time).
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import tf2_ros


class OdomTfBroadcaster(Node):
    """Republish nav_msgs/Odometry as a dynamic TF transform."""

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('odom_topic',
                               '/uav1/mavros/local_position/odom')
        self.declare_parameter('tf_parent_frame_override', '')
        self.declare_parameter('tf_child_frame_override', '')
        self.declare_parameter('use_odom_header_stamp', True)

        odom_topic = self.get_parameter('odom_topic').value
        self._parent_override = self.get_parameter(
            'tf_parent_frame_override').value
        self._child_override = self.get_parameter(
            'tf_child_frame_override').value
        self._use_header_stamp = self.get_parameter(
            'use_odom_header_stamp').value

        self._broadcaster = tf2_ros.TransformBroadcaster(self)

        self._sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, 10)

        self.get_logger().info(
            f'odom_tf_broadcaster started. Subscribed to: {odom_topic}')

    def _odom_cb(self, msg: Odometry):
        t = TransformStamped()

        if self._use_header_stamp:
            t.header.stamp = msg.header.stamp
        else:
            t.header.stamp = self.get_clock().now().to_msg()

        parent = self._parent_override or msg.header.frame_id
        child = self._child_override or msg.child_frame_id or 'base_link'

        t.header.frame_id = parent
        t.child_frame_id = child

        pos = msg.pose.pose.position
        t.transform.translation.x = pos.x
        t.transform.translation.y = pos.y
        t.transform.translation.z = pos.z

        ori = msg.pose.pose.orientation
        t.transform.rotation.x = ori.x
        t.transform.rotation.y = ori.y
        t.transform.rotation.z = ori.z
        t.transform.rotation.w = ori.w

        self._broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
