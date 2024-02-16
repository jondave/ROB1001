## ROB1001 ROS2 Python Code
This folder contains examples of Python code for ROS2.


* `colour_contours.py` - Uses the colour camera, converts the image to OpenCV images, converts to HSV, threshold the image depending on a range of colours, draws contours (outline) around selected colours, prints the mean values of the colours and publishes the images on topics. Use Rviz to show images make sure Rviz is subscribing to the correct image topics.
* `colour_detect.py` - Uses the colour camera, calculates the average colour in the image and prints the colour name to the terminal.
* `displacement.py` - Uses the robot's odometry to print to the terminal and publish the angular and linear displacement of the robot.
* `lidar_distance.py` -Uses the lidar to find the scan with the shortest distance, prints that distance to the terminal and publishes it as a float on the topic scan_closest.
* `move_square.py` - Send movement commands on the cmd_vel topic to move the robot in a predefined way, here to move in a square forward for 10 seconds then turn right for 5 seconds.
* * `speed_odom.py` - Uses the robot's odometry to print to the terminal and publish the angular and linear speed of the robot.

Instructions on how to use the code can be found on the [Wiki page](https://github.com/LCAS/ROB1001/wiki/Run-the-ROB1001-ROS-scripts).
