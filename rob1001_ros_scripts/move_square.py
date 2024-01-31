#!/usr/bin/env python3

# Written for humble

# this python script publishes movement commands on the cmd_vel topic to move the robot in a predifined way, here to move in a square forawd for 10 seconds then turn right for 5 seconds

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveSquare(Node):
    def __init__(self):
        super().__init__('move_sqaure')
        
        # Create a publisher for the cmd_vel topic
        self.robot_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.move_in_square()

    def move(self, robot_publisher, linear_x, angular_z, duration):
        print("Moving at speed X:" + str(linear_x) + " and Z:" + str(angular_z) + " for " + str(duration) + " seconds")
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z        

        start_time = time.time()
        while time.time() - start_time < duration:
            robot_publisher.publish(msg)

    def move_in_square(self):
        # Move forward for 10 seconds
        self.move(self.robot_publisher, linear_x=0.2, angular_z=0.0, duration=10.0)
        
        # Turn right for 5 seconds (adjust the angular velocity as needed)
        self.move(self.robot_publisher, linear_x=0.0, angular_z=-0.4, duration=5.0)

        # Move forward for 10 seconds
        self.move(self.robot_publisher, linear_x=0.2, angular_z=0.0, duration=10.0)

        # Turn right for 5 seconds (adjust the angular velocity as needed)
        self.move(self.robot_publisher, linear_x=0.0, angular_z=-0.4, duration=5.0)

        # Move forward for 10 seconds
        self.move(self.robot_publisher, linear_x=0.2, angular_z=0.0, duration=10.0)

        # Turn right for 5 seconds (adjust the angular velocity as needed)
        self.move(self.robot_publisher, linear_x=0.0, angular_z=-0.4, duration=5.0)

        # Move forward for 10 seconds
        self.move(self.robot_publisher, linear_x=0.2, angular_z=0.0, duration=10.0)

        # Stop the robot
        self.move(self.robot_publisher, linear_x=0.0, angular_z=0.0, duration=0.1)

def main(args=None):
    print('Starting move_square.py.')

    rclpy.init(args=args)

    move_sqaure = MoveSquare()

    rclpy.spin(move_sqaure)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_sqaure.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
