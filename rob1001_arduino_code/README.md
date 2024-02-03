## ROB1001 ARDUINO CODE
This folder contains examples of how to connect a PC and Arduino using serial communication:

* `serial_example_1` lets the user type a number using the PC keyboard, the number is sent to the Arduino. Arduino adds a unit and then sends to the PC the altered number which is finally printed in the terminal.
* `serial_example_2` lets the PC receive periodic messages from the Arduino. The messages are printed in the terminal and represent a counter.
* `serial_example_3` is similar to serial_example_2 but apart from printing the counter number in a terminal, it also publishes the counter in a ROS2 topic.
