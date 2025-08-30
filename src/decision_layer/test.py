#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32

class LaserScanToPointCloud:
    def __init__(self):
        self.pub = rospy.Publisher('/point_cloud', PointCloud, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.convert_scan)

    def convert_scan(self, scan):
        point_cloud = PointCloud()
        point_cloud.header = scan.header

        for i in range(len(scan.ranges)):
            # calculate angle and distance of each point
            angle = scan.angle_min + i * scan.angle_increment
            distance = scan.ranges[i]

            # convert polar coordinates to Cartesian coordinates
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            # add point to point cloud
            point = Point32(x, y, 0)
            point_cloud.points.append(point)

        self.pub.publish(point_cloud)

if __name__ == '__main__':
    rospy.init_node('laser_scan_to_point_cloud')
    converter = LaserScanToPointCloud()
    rospy.spin()