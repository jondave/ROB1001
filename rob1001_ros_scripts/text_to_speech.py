#!/usr/bin/env python3

# Written for humble

# This python script subscribes to a custom topic with a string-type message and generates a voice message that is reproduced by a speaker.

# Dependencies 
# pip install pyttsx3
# sudo apt update
# sudo apt install espeak
# sudo apt-get install alsa-utils

import rclpy
from rclpy.node import Node
import pyttsx3 # text-to-speech library

from std_msgs.msg import String

class text_to_speech(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        # Subscribe to the topic with the speech content
        self.subscription = self.create_subscription(String, '/text_to_speech', self.text_to_speech_callback, 10)
        # initialize speech coverter
        self.converter=pyttsx3.init("espeak") 
        self.converter.setProperty('rate',135) # set speed percent, can be more than 100
        self.converter.setProperty('volume',1)
        self.text="" # initial text is empty

    def text_to_speech_callback(self, msg):
        # read the text to be converted to speech
        if msg.data!="":
            self.text = msg.data
            # Print the speech
            print('Speech: ',self.text)
            # Generate voice message
            self.converter.say(self.text)
            self.converter.runAndWait()

        

def main(args=None):
    print('Starting text_to_speech.py')

    rclpy.init(args=args)

    tts = text_to_speech()

    rclpy.spin(tts)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tts.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
