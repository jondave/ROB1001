#!/usr/bin/env python3

# Written for humble

# this python script subscribes the RGB camera, converts to OpenCV images, converts to HSV, threshhold the image depend on range of colours, draws contours (outline) around selected colours and display the image

# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

class ColourContours(Node):
    def __init__(self):
        super().__init__('colour_contours')
        self.sub_camera = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 10)
        self.sub_camera # prevent unused variable warning

        self.br = CvBridge()

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")

        cv2.namedWindow("Image window", 1)
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "passthrough") # 'bgr8'
        except CvBridgeError as e:
            print(e)

        # Convert the image to HSV
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Create a binary (mask) image, set the values depedning on the range of colours you are looking for 
        # HSV (Hue, Saturation, Value)
        hsv_thresh = cv2.inRange(hsv_img,
                                 np.array((0, 150, 50)), # min values
                                 np.array((255, 255, 255))) # max values

        # Print the mean value of each HSV channel within the mask ranges
        print("Mean hue: " + str(cv2.mean(hsv_img[:, :, 0], mask = hsv_thresh)[0]))
        print("Mean saturation: " + str(cv2.mean(hsv_img[:, :, 1], mask = hsv_thresh)[0]))
        print("Mean value: " + str(cv2.mean(hsv_img[:, :, 2], mask = hsv_thresh)[0]))

        # Find the contours in the mask generated from the HSV image
        # The contours are outline around the shapes in the mask image
        hsv_contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        
        # in hsv_contours there are an array of individual contours (basically a polgon around the blobs in the mask)
        for c in hsv_contours:
            # This allows to compute the area (in pixels) of a contour
            a = cv2.contourArea(c)
            # and if the area is big enough, we draw the outline of the contour on the image
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)

        print('====')

        # convert the image back to RGB
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # show the image in the image window
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

def main(args=None):
    print('Starting colour_contours.py.')

    rclpy.init(args=args)

    colour_contours = ColourContours()

    rclpy.spin(colour_contours)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_contours.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()