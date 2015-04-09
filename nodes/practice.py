#!/usr/bin/env python
from projected_interface_builder.projected_interface import ProjectedInterface
from std_msgs.msg import Empty
import rospy


class Practice(ProjectedInterface):
    def __init__(self, polygon_file):
        super(Practice, self).__init__(polygon_file)
        self.order = ['2', '1', '3', '4', '6', '5', '8', '10', '7', '11', '9']
        self.set_color(self.order[0], (255, 127, 0))
        self.start()
        self.finished_pub = rospy.Publisher('~finished', Empty, latch=True)

        for polynum in range(1, 12):
            self.register_callback(str(polynum), self.click_cb)

    def click_cb(self, poly):
        if poly['uid'] == self.order[0]:
            self.reset_color(poly['uid'])
            self.publish_polygon(self.polygons[poly['uid']])
            self.order.pop(0)
            if len(self.order) > 0:
                self.set_color(self.order[0], (255, 127, 0))
                self.publish_polygon(self.polygons[self.order[0]])
            else:
                self.finished_pub.publish()

if __name__ == '__main__':
    import sys
    if len(rospy.myargv()) != 2:
        print 'Usage: practice.py interface_file'
        sys.exit(1)

    interface_file = rospy.myargv()[1]
    rospy.init_node('practice')
    interf = Practice(interface_file)
    rospy.spin()
    interf.maybe_write_changes()
