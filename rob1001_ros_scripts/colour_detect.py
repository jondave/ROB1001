#!/usr/bin/env python3

# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

class ColourDetect(Node):
    def __init__(self):
        super().__init__('colour_detect')

        # Subscribe to the camera colour image topic, the "camera_callback" is the function that is called when new data is received on the topic.
        # Make sure in the subscriber the topic name is correct as they are different for the simulation (/limo/depth_camera_link/image_raw) and real robots (/camera/color/image_raw)
        self.sub_camera = self.create_subscription(Image, '/camera/color/image_raw', self.camera_callback, 10)
        self.sub_camera # prevent unused variable warning

        # Create the publisher to publish the colour name on the topic 'colour_name'
        self.publisher_colour_name = self.create_publisher(String, 'colour_name', 10)  # create publisher for String messages

        self.br = CvBridge()

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "passthrough") # 'bgr8'
        except CvBridgeError as e:
            print(e)

        average_colour = np.mean(cv_image, axis=(0, 1))  # average colour for each channel
        #print("Average colour (BGR):", average_colour)

        colour_name = self.detect_colour(average_colour)
        print("Detected colour:", colour_name)

        # Publish the detected colour name on the "colour_name" topic
        msg = String()
        msg.data = colour_name
        self.publisher_colour_name.publish(msg)

    def detect_colour(self, avg_colour):
        # Define colour ranges for different colours
        red_range = ((0, 0, 150), (50, 50, 255))
        green_range = ((0, 150, 0), (50, 255, 50))
        blue_range = ((150, 0, 0), (255, 50, 50))
        yellow_range = ((0, 150, 150), (50, 255, 255))
        magenta_range = ((150, 0, 150), (255, 50, 255))
        cyan_range = ((0, 150, 150), (50, 255, 255))
        purple_range = ((80, 0, 80), (180, 80, 180))
        grey_range = ((80, 80, 80), (180, 180, 180))

        # Check which colour range the average colour falls into
        if self.is_in_range(avg_colour, red_range):
            return 'Red'
        elif self.is_in_range(avg_colour, green_range):
            return 'Green'
        elif self.is_in_range(avg_colour, blue_range):
            return 'Blue'
        elif self.is_in_range(avg_colour, yellow_range):
            return 'Yellow'
        elif self.is_in_range(avg_colour, magenta_range):
            return 'Magenta'
        elif self.is_in_range(avg_colour, cyan_range):
            return 'Cyan'
        elif self.is_in_range(avg_colour, purple_range):
            return 'Purple'
        elif self.is_in_range(avg_colour, grey_range):
            return 'Grey'
        else:
            return 'Unknown'

    def is_in_range(self, colour, colour_range):
        return all(colour_range[0][i] <= colour[i] <= colour_range[1][i] for i in range(3))
    

def main(args=None):
    print('Starting colour_detect.py.')

    rclpy.init(args=args)

    colour_detect = ColourDetect()

    rclpy.spin(colour_detect)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()