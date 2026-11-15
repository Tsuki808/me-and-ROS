#!/usr/bin/env python
"""Save one camera observation when diagnosing a failed visual docking stage."""
from __future__ import print_function
import os
import sys
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers

if __name__ == '__main__':
    rospy.init_node('delivery_observation', anonymous=True)
    image = rospy.wait_for_message('/inspection/image_raw', Image, timeout=10)
    path = os.path.expanduser('~/.ros/final_project/inspection.png')
    cv2.imwrite(path, CvBridge().imgmsg_to_cv2(image, 'bgr8'))
    print(path)
    print(rospy.wait_for_message('/ar_pose_marker', AlvarMarkers, timeout=5))
