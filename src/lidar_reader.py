#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan


def scan_cb(msg: LaserScan):
    rospy.loginfo(f"Length of scan {len(msg.ranges)}")
    rospy.loginfo(msg.ranges[200])


if __name__ == "__main__":
    try:
        rospy.init_node("lidar_reader")
        sub = rospy.Subscriber("/scan1", LaserScan, scan_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

