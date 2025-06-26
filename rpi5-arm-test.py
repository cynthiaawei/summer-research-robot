"""
Program is to run the arms of the robot : ]

Shoulders are controlled by a servo controlled by H-bridge L298 motor driver
which should provide 4.6-6.8V to the motors, which is accomplished
using PWM to reduce the voltage from ~12V.

The rest of arms are controlled using the ax12a actuators, which are controlled with 
serial communication through the TX pins of the adafruit feather 32u4, since the 
serial ports are currently being used for LiDAR.

"""

#import pyax12 #used to control AXA12a motors
import RPi.GPIO as GPIO




