#! /usr/bin/python3
from math import sqrt, atan2
import rospy
import motors
import imu
import ultrasonic
import numpy as np
import keyboard

# from geometry_msgs.msg import Vector3
from std_msgs.msg import String

import RPi.GPIO as gpio
import time

import sys
import signal

def signal_handler(signal, frame):  # ctrl + c -> exit program
    demo_robot.stop()
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class DemoRobot:
    def __init__(self):

        # set up relevant statuses to robot as it navigates
        self.isAvoiding = False
        self.isReorienting = False
        self.roll = 0
        self.pitch = 0
        self.theta = 0

        #
        self.distance = 0
        self.status = 'forward'

        # defines robot parts
        self.motors = motors.MotorDriver()
        self.imu = imu.IMU()
        self.ultrasonic = ultrasonic.sonar()

        # init node
        rospy.init_node('simulation', anonymous=False)

        # current pose of robot
        self.orientation_subscriber = rospy.Subscriber('/imu_orientation', String, self.update_orientation) # will only store orientation info
        self.ultrasonic_subscriber = rospy.Subscriber('/sonar_dist', String, self.sonar_info)
        self.motor_subscriber = rospy.Subscriber('/motor_status', String, self.motor_status)

        self.rate = rospy.Rate(10)

    # status updates will change the status of the robot over maze follower sequence
    # will update orientation when encountering an obstacle (will then navigate by turning 90 degrees right or left

    def update_orientation(self, data):
        self.theta = float(data.data) # convert to native float

    def sonar_info(self, data):
        self.distance = float(data.data)
        print('distance ----------------', self.distance)
        if (self.distance <= 35):
            self.isAvoiding = True
            print('is avoiding')
        else:
            self.isAvoiding = False
            print('no longer avoiding')


    def motor_status(self, data):
        self.status = data

    ## Make abilities more easy to comprehend (more straightforward)
    def moveRight(self):
        self.motors.right(0.25, power = 25)
        # Publishing our vel_msg for RVis
        self.motors.motor_publisher.publish('right')

    def moveLeft(self):
        self.motors.left(0.25, power = 25)
        self.motors.motor_publisher.publish('left')

    def moveBackwards(self):
        self.motors.backward(0.25, power= 50)
        self.motors.motor_publisher.publish('backward')

    def moveForwards(self):
        self.motors.forward(0.25, power = 50)
        self.motors.motor_publisher.publish('forward')

    def stop(self):
        self.motors.stop()
        self.motors.motor_publisher.publish('stop')

    def shutdown(self):
        print('successfully reached goal .. shutting down')

    def sonar_behavior(self):
        # ultrasonic sensor stuff
        gpio.output(self.ultrasonic.trig, False)
        time.sleep(0.1)
        gpio.output(self.ultrasonic.trig, True)
        time.sleep(0.00001)
        gpio.output(self.ultrasonic.trig, False)
        while gpio.input(self.ultrasonic.echo) == 0:
            pulse_start = time.time()
        while gpio.input(self.ultrasonic.echo) == 1:
            pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17000
        # if pulse_duration >= 0.01746:
        #     # print('time out')
        #     continue
        # elif distance > 300 or distance == 0:
        #     # print('out of range')
        #     continue
        self.distance = round(distance, 3)

    ## Example tasks that we would make robot do
    def initiateTeleopBehavior(self):
        # timestep = 20
        while (True): # must change condition so that it will complete maze course or just end after certain amount of time

            print('beginning loop')
            self.sonar_behavior()
            # print ('Distance : %f cm'%distance)

            self.ultrasonic.dist_sendor(str(self.distance)) # will send data to
            r, p, y = self.imu.sensor.euler
            self.imu.orientation_sendor(str(y))

            if keyboard.read_key() == 'w':
                self.moveForwards()
            if keyboard.read_key() == 'a':
                self.moveLeft()
            if keyboard.read_key() == 's':
                self.moveBackwards()
            if keyboard.read_key() == 'd':
                self.moveRight()

            self.rate.sleep()

        # self.stop()
        # print('stopping -- reached end of maze')

## potentially use different bug algorithms to demo to students

if __name__ == '__main__':
    ## Put your functions here to start
    demo_robot = DemoRobot()
    while not rospy.is_shutdown():
        demo_robot.initiateTeleopBehavior()
        rospy.spin()

    while rospy.is_shutdown():
        demo_robot.stop()
