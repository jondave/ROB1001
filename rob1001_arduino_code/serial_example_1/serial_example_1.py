# Importing Libraries 
import serial 
import time 
# Change for your com port, e.g.: "COM8:", "/dev/ttyUSB0", etc.
arduino = serial.Serial(port='COM3', baudrate=115200, timeout=.1) 

def write_read(x): 
    arduino.write(bytes(x, 'utf-8')) 
    time.sleep(0.05) 
    data = arduino.readline() 
    return data 

while ( True ): 
    num = input("Enter a number: ") # Taking input from user 
    value_byte = write_read(num)
    value_str= value_byte.decode("utf-8") #from byte to string
    print("Number received: ",value_str) # printing the value 

arduino.close()