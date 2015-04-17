#!/usr/bin/env python

import rospy
import tf
from PyKDL import Rotation
import dynamic_reconfigure.client

# find the rotation between the glass frame and the face detection frame, and
# rotate the face detection frame by its inverse

if __name__ == '__main__':
    rospy.init_node('frame_adjust')
    client = dynamic_reconfigure.client.Client(rospy.myargv()[1])
    listener = tf.TransformListener()
    listener.waitForTransform('face_detection', 'glass', rospy.Time(), rospy.Duration(10))
    trans, rot = listener.lookupTransform('glass', 'face_detection', rospy.Time(0))
    rot = Rotation.Quaternion(*rot)

    r, p, y = rot.Inverse().GetRPY()

    config = client.get_configuration()
    config.yaw -= y
    # config.pitch -= p
    # config.roll -= r

    client.update_configuration(config)
