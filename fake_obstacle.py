#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


def callback(data):
    global pub
    scan = LaserScan()
    scan = data
    data_list = list(data.ranges)
    data_list[135:225] = [2 for i in range(90)]
    scan_list = data_list
    scan.ranges = tuple(scan_list)
    pub.publish(scan)


def main():
    global pub
    rospy.init_node("fake_obstacle_creater")
    pub = rospy.Publisher("/fakescan", LaserScan, queue_size=10)

    sub = rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()


if __name__ == "__main__":
    main()
