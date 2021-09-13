#! /usr/bin/python3
from math import sqrt, atan2
import rospy
import motors
import imu
import ultrasonic
import numpy as np

# from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, String

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
        self.orientation_subscriber = rospy.Subscriber('/imu_orientation', Float32, self.update_orientation) # will only store orientation info
        self.ultrasonic_subscriber = rospy.Subscriber('/sonar_dist', Float32, self.sonar_info)
        self.motor_subscriber = rospy.Subscriber('/motor_status', String, self.motor_status)

        self.rate = rospy.Rate(10)

    # status updates will change the status of the robot over maze follower sequence
    # will update orientation when encountering an obstacle (will then navigate by turning 90 degrees right or left

    def update_orientation(self, data):
        self.theta = np.float32(data).item() # convert to native float

    def sonar_info(self, data):
        self.distance = np.float32(data).item()
        if (self.distance >= 4.0):
            self.isAvoiding = True
        else:
            self.isAvoiding = False


    def motor_status(self, data):
        self.status = data

    ## Make abilities more easy to comprehend (more straightforward)
    def moveRight(self):
        self.motors.right()
        # Publishing our vel_msg for RVis
        self.motors.motor_publisher.publish('right')

    def moveLeft(self):
        self.motors.left()
        self.motors.motor_publisher.publish('left')

    def moveBackwards(self):
        self.motors.backward()
        self.motors.motor_publisher.publish('backward')

    def moveForwards(self):
        self.motors.forward(2)
        self.motors.motor_publisher.publish('forward')

    def stop(self):
        self.motors.stop()
        self.motors.motor_publisher.publish('stop')

    def shutdown(self):
        print('successfully reached goal .. shutting down')

    ## Example tasks that we would make robot do
    def initiateMazeBehavior(self):
        timestep = 20
        while (True): # must change condition so that it will complete maze course or just end after certain amount of time

            print('beginning loop')
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
            if pulse_duration >= 0.01746:
                # print('time out')
                continue
            elif distance > 300 or distance == 0:
                # print('out of range')
                continue
            distance = round(distance, 3)
            # print ('Distance : %f cm'%distance)

            self.ultrasonic.dist_sendor(distance) # will send data to

            r, p, y = self.imu.sensor.euler
            self.imu.orientation_sendor(y)
            # ultrasonic.r.sleep()

            while (self.isAvoiding):
                print('initating avoidance behavior')
                while (self.theta > -1.5708):
                    self.moveBackwards()  # intended to rotate robot away from obstacle
                    self.moveRight()  # slight movement backward along the obstacle
                    # Angular velocity in the z-axis.
                    self.rate.sleep()

                while (self.theta < 3.14):
                    self.moveBackwards()  # intended to rotate robot away from obstacle
                    self.moveLeft()  # slight movement backward along the obstacle
                    # Angular velocity in the z-axis.
                    self.rate.sleep()

            self.moveForwards()
            print('moving forward')
            timestep = timestep + 1 # a way to ensure that robot isn't moving forward indefinitely
            self.rate.sleep()

        self.stop()
        print('stopping -- reached end of maze')

## potentially use different bug algorithms to demo to students

if __name__ == '__main__':
    ## Put your functions here to start
    demo_robot = DemoRobot()
    while not rospy.is_shutdown():
        demo_robot.initiateMazeBehavior()
        rospy.spin()

    while rospy.is_shutdown():
        demo_robot.stop()
