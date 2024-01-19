#!/usr/bin/env python3

# Written for humble

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class LidarClosest(Node):
    def __init__(self):
        super().__init__('lidar_closest')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher_scan_closest = self.create_publisher(Float32, 'scan_closest', 10)


    def lidar_callback(self, msg):
        if not msg.ranges:
            return

        # Find the closest distance from the laser scan data
        closest_distance = min(msg.ranges)

        # Print the closest distance to the terminal
        #print('Closest Distance: {}'.format(closest_distance))

        # Publish the closest distance on the "scan_closest" topic
        scan_closest_msg = Float32()
        scan_closest_msg.data = closest_distance
        self.publisher_scan_closest.publish(scan_closest_msg)
    

def main(args=None):
    print('Starting colour_detect.py.')

    rclpy.init(args=args)

    lidar_closest = LidarClosest()

    rclpy.spin(lidar_closest)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_closest.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()