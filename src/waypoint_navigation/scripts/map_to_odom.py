#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from tf import transformations

from tf.listener import TransformListener
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Header
import numpy as np
from tf2_msgs.msg import TFMessage

rospy.init_node('map_to_odom')

listener = TransformListener()
publisher = TransformBroadcaster()
rate = rospy.Rate(100)
rospy.sleep(5)

time = rospy.Time.now()
listener.waitForTransform(target_frame='camera_link_slam', source_frame='map', time=time, timeout=rospy.Duration(1))
listener.waitForTransform(target_frame='odom', source_frame='camera_link', time=time, timeout=rospy.Duration(1))

trans = None
rot = None
camera_link_slam_stamp = None
camera_link_stamp = None

def set_stamp(message):  # type: (TFMessage) -> None
    global camera_link_slam_stamp, camera_link_stamp
    if message.transforms[0].child_frame_id == 'camera_link_slam':
        camera_link_slam_stamp = message.transforms[0].header.stamp
    if message.transforms[0].child_frame_id == 'camera_link':
        camera_link_stamp = message.transforms[0].header.stamp

sub = rospy.Subscriber('/tf', TFMessage, set_stamp)

while not rospy.is_shutdown():
    try:
        time = rospy.Time.now()
        # stamp = time - rospy.Duration(0.4)
        Q = listener.asMatrix(target_frame='camera_link_slam', hdr=Header(frame_id='map', stamp=camera_link_slam_stamp))
        P_inv = listener.asMatrix(target_frame='odom', hdr=Header(frame_id='camera_link', stamp=camera_link_stamp))
        res = np.matmul(P_inv, Q)
        angle, direc, point = transformations.rotation_from_matrix(res)
        rot = transformations.rotation_matrix(angle, direc, point)
        
        trans = transformations.translation_from_matrix(res)
        rot = transformations.quaternion_from_matrix(rot)
    except Exception as err:
        print(err)
    if trans is not None and rot is not None:
        publisher.sendTransform(
            translation=trans,
            rotation=rot,
            time=time,
            child='odom',
            parent='map',
        )
    rate.sleep()
