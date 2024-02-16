#!/usr/bin/env python3

# Written for humble

# this python script subscribes to the odometry topic and prints to the terminal and publishes the position and orientation of the robot.

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32

class Displacement(Node):
    def __init__(self):
        super().__init__('displacement')

        # Subscribe to the odom topic
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Create publishers for float topics
        self.position_publisher = self.create_publisher(Float32, 'position', 10)
        self.orientation_publisher = self.create_publisher(Float32, 'orientation', 10)

    def odom_callback(self, msg):
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        orientation_z = msg.pose.pose.orientation.z

        # Print position and orientation
        print("Position (x, y): " + str(position_x) + ", " + str(position_y))
        print("Orientation (z): " + str(orientation_z))

        # Publish the position and orientation to the float topics
        position_msg = Float32()
        position_msg.data = position_x
        self.position_publisher.publish(position_msg)

        position_msg.data = position_y
        self.position_publisher.publish(position_msg)

        orientation_msg = Float32()
        orientation_msg.data = orientation_z
        self.orientation_publisher.publish(orientation_msg)
    

def main(args=None):
    print('Starting displacement.py.')

    rclpy.init(args=args)

    displacement = Displacement()

    rclpy.spin(displacement)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    displacement.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
