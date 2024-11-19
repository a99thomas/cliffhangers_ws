#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def laser_scan_publisher():
    rospy.init_node('laser_scan_publisher', anonymous=True)
    scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
    
    num_readings = 100
    laser_frequency = 40.0
    ranges = [0] * num_readings
    intensities = [0] * num_readings

    count = 0
    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        # Generate some fake data for our laser scan
        for i in range(num_readings):
            ranges[i] = count
            intensities[i] = 100 + count
        
        scan_time = rospy.Time.now()
        
        # Populate the LaserScan message
        scan = LaserScan()
        scan.header.stamp = scan_time
        scan.header.frame_id = "laser_frame"
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = 3.14 / num_readings
        scan.time_increment = (1 / laser_frequency) / num_readings
        scan.range_min = 0.0
        scan.range_max = 100.0

        scan.ranges = ranges
        scan.intensities = intensities

        scan_pub.publish(scan)
        count += 1
        r.sleep()

if __name__ == '__main__':
    try:
        laser_scan_publisher()
    except rospy.ROSInterruptException:
        pass


