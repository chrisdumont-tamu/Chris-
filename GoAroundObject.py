# Sample for Sabertooth Simplified Serial
# Test w/ RPi3B+ -> USB to TTL 5V Cable -> Sabertooth 2X12
# USB to TTL Cable Hookup to Sabertooth: GND -> 0V, TX -> S1
# DIP Config (9600 Buad): 1UP - 2DOWN - 3 UP - 4DOWN - 5UP - 6UP

# Basic imports
# Serial only works on RPi

import time
import serial

#Requirments for the code to work
#Learn to understand this
port = "/dev/ttyUSB0"  #find available USB using "ls /dev/*USB* in terminal"
ser = serial.Serial()
ser.port = port
ser.close()
 
ser.baudrate = 9600
ser.bytesize = serial.EIGHTBITS
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE
ser.timeout = 1
ser.open()
ser.flushInput()
ser.flushOutput()

# M1: 1 - 127; 1 = full reverse, 64 = stop, 127 = full forward
# M2: 128 - 255; 128 = full reverse, 192 = stop, 255 = full forward
# M1 & M2; 0 = stop
# Stop +/- 6 is good for slow movement
# Avoid going full speed

pause=1

def stop():
    ser.write(bytes([0]))
    time.sleep(pause)


def forward_2ft():
    print("forward")
    ser.write(bytes([70]))
    ser.write(bytes([198]))
    time.sleep(3)
    ser.write(bytes([0]))
    stop()    

def forward_1ft():
    print("forward")
    ser.write(bytes([70]))
    ser.write(bytes([198]))
    time.sleep(1.5)
    ser.write(bytes([0]))
    stop()

def turn_left_45():
    print("turning left")
    ser.write(bytes([58]))
    ser.write(bytes([198]))
    time.sleep(pause)
    stop()

def turn_right_45():
    print("turning right")
    ser.write(bytes([70]))
    ser.write(bytes([186]))
    time.sleep(pause)
    stop()

def around_left():

    forward_2ft()
    turn_left_45()
    forward_1ft()
    turn_right_45()
    forward_1ft()
    turn_right_45()
    forward_1ft()
    turn_left_45()
    forward_2ft()

def around_right():

    forward_2ft()
    turn_right_45()
    forward_1ft()
    turn_left_45()
    forward_1ft()
    turn_left_45()
    forward_1ft()
    turn_right_45()
    forward_2ft()

def main():
    obstacle_location=input("do I need to go left or right? (1 for Left, 2 for Right) : ")

    if obstacle_location==1:
            around_left()
    elif obstacle_location==2:
            around_right()
    stop()
    print("Did it Bitch")
main()
