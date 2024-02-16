#!/usr/bin/env python3

# Written for humble

# this python script subscribes to the odometry topic and prints to the terminal and publishes the angular and linear displacement of the robot.

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class SpeedOdom(Node):
    def __init__(self):
        super().__init__('speed_odom')

        # Subscribe to the odom topic
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Create publishers for float topics
        self.linear_speed_publisher = self.create_publisher(Float32, 'speed_linear', 10)
        self.angular_speed_publisher = self.create_publisher(Float32, 'speed_angular', 10)

    def odom_callback(self, msg):

        linear_speed = msg.twist.twist.linear.x
        angular_speed = msg.twist.twist.angular.z

        # Print speeds
        print("Linear speed: " + str(linear_speed))
        print("Angular speed: " + str(angular_speed))

        # Publish the speeds to the float topics
        linear_speed_msg = Float32()
        linear_speed_msg.data = linear_speed
        self.linear_speed_publisher.publish(linear_speed_msg)

        angular_speed_msg = Float32()
        angular_speed_msg.data = angular_speed
        self.angular_speed_publisher.publish(angular_speed_msg)
    

def main(args=None):
    print('Starting speed_odom.py.')

    rclpy.init(args=args)

    speed_odom = SpeedOdom()

    rclpy.spin(speed_odom)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    speed_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()