#!/usr/bin/env python3

# Written for humble

# this python script subscribes to the odemetry topic and prints to the terminal and publishes the angular and linear displacement of the robot.

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class Displacement(Node):
    def __init__(self):
        super().__init__('displacement')

        # Subscribe to the odom topic
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Create publishers for float topics
        self.linear_displacement_publisher = self.create_publisher(Float32, 'displacement_linear', 10)
        self.angular_displacement_publisher = self.create_publisher(Float32, 'displacement_angular', 10)

    def odom_callback(self, msg):

        linear_displacement = msg.twist.twist.linear.x
        angular_displacement = msg.twist.twist.angular.z

        # Print displacements
        print("Linear displacement: " + str(linear_displacement))
        print("Angular displacement: " + str(angular_displacement))

        # Publish the displacements to the float topics
        linear_displacement_msg = Float32()
        linear_displacement_msg.data = linear_displacement
        self.linear_displacement_publisher.publish(linear_displacement_msg)

        angular_displacement_msg = Float32()
        angular_displacement_msg.data = angular_displacement
        self.angular_displacement_publisher.publish(angular_displacement_msg)
    

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