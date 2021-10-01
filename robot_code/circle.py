from math import sqrt, atan2
import rospy
import motors
import imu
import ultrasonic
import numpy as np

# from geometry_msgs.msg import Vector3
from std_msgs.msg import String

import RPi.GPIO as gpio
import time

import sys
import signal

motors = motors.MotorDriver()

def signal_handler(signal, frame):  # ctrl + c -> exit program
    motors.stop()
    print('You pressed Ctrl+C!')
    sys.exit(0)

def circle():
    motors.forward(10)
    motors.right(100)

def sinosoid():
    for i in range(4):
        motors.forward(10)
        motors.right(10)
        motors.left(10)

if __name__ == '__main__':
    circle()
    # sinosoid()