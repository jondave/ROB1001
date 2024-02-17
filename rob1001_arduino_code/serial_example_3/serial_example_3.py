#!/usr/bin/env python3

# Dependencies
# pip install pyserial

# Require user to give Port permission
# sudo chmod 666 <port_name> 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class com_class(Node):
    def __init__(self): 
        super().__init__('ros_arduino_communication')
        #Variables
        self.alert="none" # LED signal
        self.counter="" # counter in str
        #Serial communication with arduino
        self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=57600, timeout=.1)
        self.startMarker = 60 #<
        self.endMarker = 62 #>
        #Setup ROS subscriber
        self.subscriber_ros = self.create_subscription(String, '/alert', self.visual_alerts_Callback, 10)
        #Setup ROS publisher
        self.publisher_ros = self.create_publisher(String, '/counter', 10)
        # Create a timer publisher which is run every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.start_time = time.time()
        
    
    def timer_callback(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        self.read_from_ros()
        ros_msg = String()
        ros_msg.data = self.counter
        self.publisher_ros.publish(ros_msg)
    
    def visual_alerts_Callback(self, msg):
        if msg.data=="red" or msg.data=="green" or msg.data=="yellow":
            self.alert = msg.data
        else:
           self.alert = "none"
    
    def read_from_ros(self):
        data = []
        data.append("<"+self.alert+">")
        self.send_from_ros(data)
    
    def waitForArduino(self):
       # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
       # it also ensures that any bytes left over from a previous message are discarded
        msg = ""
        while msg.find("Arduino is ready") == -1:   
          while self.ser.inWaiting() == 0:
            pass
          msg = self.recvFromArduino()
          print(msg)
    
    def sendToArduino(self,sendStr):
      self.ser.write(sendStr.encode())
       
    def recvFromArduino(self):
      ck = ""
      x = "z" # any value that is not an end- or startMarker
      byteCount = -1 # to allow for the fact that the last increment will be one too many    
      # wait for the start character
      while  ord(x) != self.startMarker: 
        x = self.ser.read().decode()    
      # save data until the end marker is found
      while ord(x) != self.endMarker:
        if ord(x) != self.startMarker:
          ck = ck + x 
          byteCount += 1
        x = self.ser.read().decode()     
      return(ck)
    
    def send_from_ros(self,td):
      numLoops = len(td)
      waitingForReply = False
      n = 0
      while n < numLoops:   
        teststr = td[n]
        if waitingForReply == False:
          self.sendToArduino(teststr)
          print("Sent from PC: " + teststr)
          waitingForReply = True
        if waitingForReply == True:  
          while self.ser.inWaiting() == 0:
            pass
        
          dataRecvd = self.recvFromArduino()
          n += 1
          waitingForReply = False
    
          print("===============")       
          self.counter=dataRecvd
          print("Number Received: " + self.counter)
        time.sleep(0.005) # to avoid lose message

def main(args=None):
    print('Starting ros_arduino_communication.py')
    # Initialize our node       
    rclpy.init(args=args)
    
    com_ros=com_class()
    
    rclpy.spin(com_ros)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    com_ros.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
