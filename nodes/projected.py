#!/usr/bin/env python
import roslib; roslib.load_manifest('projected_interface_builder')
from projected_interface_builder.projected_interface import ProjectedInterface
from projected_interface_builder.data_structures import PolygonInfo
import rospy
from vlc.srv import *

class VideoControl(ProjectedInterface):
    def __init__(self, polygon_file):
        super(VideoControl, self).__init__(polygon_file)
        self.play       = rospy.ServiceProxy('/play', Play)
        self.pause      = rospy.ServiceProxy('/pause', Pause)
        self.back10     = rospy.ServiceProxy('/back10', Back10)
        self.forward10  = rospy.ServiceProxy('/forward10', Forward10)
        self.vol_up     = rospy.ServiceProxy('/vol_up', VolUp)
        self.vol_dn     = rospy.ServiceProxy('/vol_dn', VolDn)
        self.mute       = rospy.ServiceProxy('/toggle_mute', MuteToggle)

        self.register_callback('ff10',   self.cb_forward10)
        self.register_callback('mute',   self.cb_mute)
        self.register_callback('pause',  self.cb_pause)
        self.register_callback('play',   self.cb_play)
        self.register_callback('rew10',  self.cb_rew10)
        self.register_callback('volup',  self.cb_volup)
        self.register_callback('voldn',  self.cb_voldn)

    def cb_forward10(self, poly):
        self.forward10()

    def cb_mute(self, poly):
        self.mute()

    def cb_pause(self, poly):
        self.pause()

    def cb_play(self, poly):
        self.play()

    def cb_rew10(self, poly):
        self.back10()

    def cb_volup(self, poly):
        self.vol_up()

    def cb_voldn(self, poly):
        self.vol_dn()


if __name__ == '__main__':
    import sys
    if len(rospy.myargv()) != 2:
        print 'Usage: projected.py interface_file'
        sys.exit(1)

    interface_file = rospy.myargv()[1]
    rospy.init_node('tv_interface')
    interf = VideoControl(interface_file)
    interf.start()
    rospy.spin()
    interf.maybe_write_changes()