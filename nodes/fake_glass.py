#!/usr/bin/env rosh

pose_msg = msg.geometry_msgs.PoseStamped()
pose_msg.header.frame_id = 'glass'

rostype(topics.android.pose, msg.geometry_msgs.PoseStamped)

r = Rate(10)
while ok():
    pose_msg.header.stamp = now()
    topics.android.pose(pose_msg)
    r.sleep()