#!/usr/bin/env python3

# Written for humble

# this python script subscribes the RGB camera, converts to OpenCV images, converts to HSV, threshold the image depend on range of colours, draws contours (outline) around selected colours and publishes the image on a new image topic

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

        # Subscribe to the camera colour image topic, the "camera_callback" is the function that is called when new data is received on the topic.
        # Make sure in the subscriber the topic name is correct as they are different for the simulation (/limo/depth_camera_link/image_raw) and real robots (/camera/color/image_raw)
        self.sub_camera = self.create_subscription(Image, '/camera/color/image_raw', self.camera_callback, 10)
        
        # Create the publishers to publish the images from OpenCV
        self.pub_cv_hsv = self.create_publisher(Image, 'open_cv_image/hsv', 10) # publish the hsv image.
        self.pub_cv_thresh = self.create_publisher(Image, 'open_cv_image/hsv_thresh', 10) # publish the threshold image based on the range of colours used.
        self.pub_cv_contours = self.create_publisher(Image, 'open_cv_image/contours', 10) # publish the image with the contours drawn around the coloured objects.
        self.sub_camera # prevent unused variable warning

        self.br = CvBridge()

    # this funciton is run when new data is recieved on the camera image topic.
    def camera_callback(self, data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "passthrough") # 'bgr8'
        except CvBridgeError as e:
            print(e)

        # Convert the image to HSV
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Publish the HSV image
        try:
            ros_hsv_msg = self.br.cv2_to_imgmsg(hsv_img, "bgr8")
            ros_hsv_msg.header = data.header  # Preserve the header
            self.pub_cv_hsv.publish(ros_hsv_msg)
        except CvBridgeError as e:
            print(e)

        # Create a binary (mask) image, set the values depending on the range of colours you are looking for 
        # HSV (Hue, Saturation, Value)
        hsv_thresh = cv2.inRange(hsv_img,
                                 np.array((0, 150, 50)), # min values
                                 np.array((255, 255, 255))) # max values

        # Publish the threshold image
        try:
            ros_thresh_msg = self.br.cv2_to_imgmsg(hsv_thresh, "mono8")
            ros_thresh_msg.header = data.header  # Preserve the header
            self.pub_cv_thresh.publish(ros_thresh_msg)
        except CvBridgeError as e:
            print(e)

        # Find the contours in the mask generated from the HSV image
        # The contours are outline around the shapes in the mask image
        hsv_contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        
        # in hsv_contours there are an array of individual contours (basically a polygon around the blobs in the mask)
        for c in hsv_contours:
            # This allows to compute the area (in pixels) of a contour
            a = cv2.contourArea(c)
            # and if the area is big enough, we draw the outline of the contour on the image
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0), 5)

        # convert the image back to BGR for publishing
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # Publish the contours image
        try:
            ros_contours_msg = self.br.cv2_to_imgmsg(cv_image, "bgr8")
            ros_contours_msg.header = data.header  # Preserve the header
            self.pub_cv_contours.publish(ros_contours_msg)
        except CvBridgeError as e:
            print(e)

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