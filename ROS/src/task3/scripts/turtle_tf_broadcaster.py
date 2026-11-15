#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose


def main():
    rospy.init_node("turtle_tf_broadcaster")
    turtle_name = rospy.get_param("~turtle_name")
    # 两个节点各持有一个广播器，避免在每次位姿回调中重新创建。
    broadcaster = tf2_ros.TransformBroadcaster()

    def broadcast_pose(pose):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = turtle_name
        transform.transform.translation.x = pose.x
        transform.transform.translation.y = pose.y
        transform.transform.translation.z = 0.0
        quaternion = tf_conversions.transformations.quaternion_from_euler(
            0.0, 0.0, pose.theta,
        )
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        broadcaster.sendTransform(transform)

    subscriber = rospy.Subscriber(
        "/" + turtle_name + "/pose", Pose, broadcast_pose, queue_size=10,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
