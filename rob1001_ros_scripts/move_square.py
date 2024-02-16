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
        self.movement_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a timer publisher which is run every 0.1 secnods
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.start_time = time.time()

    def timer_callback(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        twist_msg = Twist()

        if elapsed_time <= 10:  # Move forward at 0.2 m/s for 10 seconds
            print("Moving at speed X: 0.2 m/s and Z: 0.0 rad/s for 10 seconds")
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.0
        elif elapsed_time <= 15:  # Turn right at 0.4 rad/s for 5 seconds
            print("Moving at speed X: 0.0 m/s and Z: 0.4 rad/s for 5 seconds")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -0.4
        elif elapsed_time <= 25:  # Move forward at 0.2 m/s for 10 seconds
            print("Moving at speed X: 0.2 m/s and Z: 0.0 rad/s for 10 seconds")
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.0
        elif elapsed_time <= 30:  # Turn right at 0.4 rad/s for 5 seconds
            print("Moving at speed X: 0.0 m/s and Z: 0.4 rad/s for 5 seconds")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -0.4
        elif elapsed_time <= 40:  # Move forward at 0.2 m/s for 10 seconds
            print("Moving at speed X: 0.2 m/s and Z: 0.0 rad/s for 10 seconds")
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.0
        elif elapsed_time <= 45:  # Turn right at 0.4 rad/s for 5 seconds
            print("Moving at speed X: 0.0 m/s and Z: 0.4 rad/s for 5 seconds")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -0.4
        elif elapsed_time <= 55:  # Move forward at 0.2 m/s for 10 seconds
            print("Moving at speed X: 0.2 m/s and Z: 0.0 rad/s for 10 seconds")
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.0
        else: # Stop robot
            print("Robot stopped")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        self.movement_publisher.publish(twist_msg)

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
